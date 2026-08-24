#!/usr/bin/env python3
"""
multi_drone_offboard_mission.py

Robust 3-drone concurrent coverage mission.
"""

import os
import csv
import time
import math
import asyncio
from typing import Dict, Tuple

from mavsdk import System
from mavsdk.offboard import OffboardError, PositionNedYaw


# ---- Configuration -----------------------------------------------------
DRONES = [
    {
        "id": 1,
        "port": 14550,
        "grpc_port": 50051,
        "waypoints_file": os.path.expanduser("~/coverage_map/waypoints_drone1.csv"),
        "log_file": os.path.expanduser("~/coverage_map/actual_path_drone1.csv"),
    },
    {
        "id": 2,
        "port": 14560,
        "grpc_port": 50052,
        "waypoints_file": os.path.expanduser("~/coverage_map/waypoints_drone2.csv"),
        "log_file": os.path.expanduser("~/coverage_map/actual_path_drone2.csv"),
    },
    {
        "id": 3,
        "port": 14570,
        "grpc_port": 50053,
        "waypoints_file": os.path.expanduser("~/coverage_map/waypoints_drone3.csv"),
        "log_file": os.path.expanduser("~/coverage_map/actual_path_drone3.csv"),
    },
]

ACCEPTANCE_RADIUS_M = 1.2
COLLISION_THRESHOLD_M = 5.0
HOLD_CHECK_INTERVAL_S = 1.0
CONNECTION_TIMEOUT_S = 25
COLLISION_GRACE_PERIOD_S = 15.0
STAGGER_DELAY_S = 2.5          # delay between starting each drone


def log(drone_id: int, message: str) -> None:
    print(f"Drone {drone_id}: {message}")


class SharedState:
    def __init__(self):
        self.global_positions: Dict[int, Tuple[float, float]] = {}
        self.hold_flags: Dict[int, bool] = {d["id"]: False for d in DRONES}


def haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    R = 6371000.0
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)
    a = (math.sin(dphi / 2) ** 2 +
         math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2) ** 2)
    return 2 * R * math.asin(math.sqrt(a))


class MultiDroneWaypointFollower:
    def __init__(self, drone_id: int, connection_url: str, grpc_port: int,
                 waypoints_file: str, log_file: str, state: SharedState):
        self.drone_id = drone_id
        self.connection_url = connection_url
        self.grpc_port = grpc_port
        self.waypoints_file = waypoints_file
        self.log_file = log_file
        self.state = state
        self.drone = System(port=grpc_port)
        self.is_logging = False

    # ------------------------------------------------------------------
    async def connect_and_check_health(self) -> bool:
        log(self.drone_id,
            f"Connecting on {self.connection_url} (gRPC {self.grpc_port})...")

        await self.drone.connect(system_address=self.connection_url)

        try:
            async def _wait_connected():
                async for s in self.drone.core.connection_state():
                    if s.is_connected:
                        return True
            await asyncio.wait_for(_wait_connected(), timeout=CONNECTION_TIMEOUT_S)
            log(self.drone_id, "Connection SUCCESSFUL.")
        except asyncio.TimeoutError:
            log(self.drone_id, "Connection FAILED (timeout).")
            return False

        log(self.drone_id, "Waiting for vehicle to become armable...")
        try:
            async def _wait_armable():
                async for h in self.drone.telemetry.health():
                    # We only require is_armable (home_position_ok is False in your SITL)
                    if h.is_armable:
                        return True
            await asyncio.wait_for(_wait_armable(), timeout=CONNECTION_TIMEOUT_S)
            log(self.drone_id, "Vehicle is armable – ready.")
            return True
        except asyncio.TimeoutError:
            log(self.drone_id, "FAILED: vehicle never became armable.")
            return False

    # ------------------------------------------------------------------
    def _load_waypoints(self, filepath: str):
        waypoints = []
        if not os.path.exists(filepath):
            log(self.drone_id, f"ERROR: waypoint file not found → {filepath}")
            return waypoints

        with open(filepath, mode="r") as f:
            reader = csv.DictReader(f)
            for row in reader:
                waypoints.append({
                    "seq": int(row["seq"]),
                    "x_local": float(row["x_local_m"]),
                    "y_local": float(row["y_local_m"]),
                    "z": float(row["z_m"]),
                })
        return waypoints

    # ------------------------------------------------------------------
    async def run_mission(self):
        waypoints = self._load_waypoints(self.waypoints_file)
        if not waypoints:
            log(self.drone_id, "No waypoints – skipping.")
            return

        self.is_logging = True
        logger_task = asyncio.create_task(self._log_telemetry())
        tracker_task = asyncio.create_task(self._track_global_position())

        try:
            # Extra settle time after health check
            await asyncio.sleep(2.0)

            log(self.drone_id, "Arming...")
            await self.drone.action.arm()
            await asyncio.sleep(2.0)

            # Explicit takeoff altitude helps reliability
            try:
                await self.drone.action.set_takeoff_altitude(10.0)
            except Exception:
                pass

            log(self.drone_id, "Taking off...")
            await self.drone.action.takeoff()
            await asyncio.sleep(10)          # let them climb

            # Initial setpoint required before offboard.start()
            first = waypoints[0]
            await self.drone.offboard.set_position_ned(
                PositionNedYaw(
                    north_m=first["x_local"],
                    east_m=first["y_local"],
                    down_m=-first["z"],
                    yaw_deg=0.0,
                )
            )

            log(self.drone_id, "Starting OFFBOARD...")
            try:
                await self.drone.offboard.start()
            except OffboardError as e:
                log(self.drone_id, f"Offboard start failed: {e._result.result}. Landing.")
                await self.drone.action.land()
                return

            log(self.drone_id, f"Executing {len(waypoints)} waypoints...")

            for idx, wp in enumerate(waypoints):
                while self.state.hold_flags.get(self.drone_id, False):
                    await asyncio.sleep(HOLD_CHECK_INTERVAL_S)

                target = PositionNedYaw(
                    north_m=wp["x_local"],
                    east_m=wp["y_local"],
                    down_m=-wp["z"],
                    yaw_deg=0.0,
                )
                await self.drone.offboard.set_position_ned(target)

                while True:
                    if self.state.hold_flags.get(self.drone_id, False):
                        await asyncio.sleep(HOLD_CHECK_INTERVAL_S)
                        continue

                    async for ned in self.drone.telemetry.position_velocity_ned():
                        dn = ned.position.north_m - wp["x_local"]
                        de = ned.position.east_m  - wp["y_local"]
                        dd = ned.position.down_m  - (-wp["z"])
                        dist = math.sqrt(dn*dn + de*de + dd*dd)
                        break

                    if dist <= ACCEPTANCE_RADIUS_M:
                        log(self.drone_id,
                            f"Reached WP {idx+1}/{len(waypoints)} (dist {dist:.2f} m)")
                        break
                    await asyncio.sleep(0.15)

            log(self.drone_id, "All waypoints done.")

            try:
                await self.drone.offboard.stop()
            except OffboardError:
                pass

            await self.drone.action.land()

            async for armed in self.drone.telemetry.armed():
                if not armed:
                    log(self.drone_id, "Disarmed – mission complete.")
                    break
                await asyncio.sleep(1.0)

        except Exception as e:
            log(self.drone_id, f"Mission exception: {e}")
            try:
                await self.drone.action.land()
            except Exception:
                pass

        finally:
            self.is_logging = False
            try:
                await logger_task
            except Exception:
                pass
            tracker_task.cancel()
            try:
                await tracker_task
            except asyncio.CancelledError:
                pass

    # ------------------------------------------------------------------
    async def _log_telemetry(self):
        os.makedirs(os.path.dirname(self.log_file), exist_ok=True)
        with open(self.log_file, mode="w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["timestamp", "latitude", "longitude",
                             "abs_altitude", "rel_altitude"])
            while self.is_logging:
                try:
                    async for pos in self.drone.telemetry.position():
                        writer.writerow([
                            f"{time.time():.3f}",
                            f"{pos.latitude_deg:.8f}",
                            f"{pos.longitude_deg:.8f}",
                            f"{pos.absolute_altitude_m:.2f}",
                            f"{pos.relative_altitude_m:.2f}",
                        ])
                        f.flush()
                        break
                except Exception as e:
                    log(self.drone_id, f"Telemetry log error: {e}")
                await asyncio.sleep(0.5)

    async def _track_global_position(self):
        while True:
            try:
                async for pos in self.drone.telemetry.position():
                    self.state.global_positions[self.drone_id] = (
                        pos.latitude_deg, pos.longitude_deg)
            except Exception as e:
                log(self.drone_id, f"GPS tracker error: {e}")
                await asyncio.sleep(1.0)


async def collision_monitor(state: SharedState):
    drone_ids = [d["id"] for d in DRONES]
    log(0, f"Collision monitor waiting {COLLISION_GRACE_PERIOD_S}s (takeoff grace)...")
    await asyncio.sleep(COLLISION_GRACE_PERIOD_S)
    log(0, "Collision monitor ACTIVE.")

    while True:
        await asyncio.sleep(HOLD_CHECK_INTERVAL_S)
        for i in range(len(drone_ids)):
            for j in range(i + 1, len(drone_ids)):
                id_a, id_b = drone_ids[i], drone_ids[j]
                pos_a = state.global_positions.get(id_a)
                pos_b = state.global_positions.get(id_b)
                if pos_a is None or pos_b is None:
                    continue

                dist = haversine_m(pos_a[0], pos_a[1], pos_b[0], pos_b[1])
                lower_id = min(id_a, id_b)
                higher_id = max(id_a, id_b)

                if dist < COLLISION_THRESHOLD_M:
                    if not state.hold_flags[higher_id]:
                        log(higher_id,
                            f"WITHIN {dist:.1f} m of Drone {lower_id} → HOLDING")
                    state.hold_flags[higher_id] = True
                else:
                    if state.hold_flags[higher_id]:
                        log(higher_id,
                            f"Clear of Drone {lower_id} ({dist:.1f} m) → RESUMING")
                    state.hold_flags[higher_id] = False


async def main():
    state = SharedState()

    followers = [
        MultiDroneWaypointFollower(
            drone_id=d["id"],
            connection_url=f"udpin://0.0.0.0:{d['port']}",
            grpc_port=d["grpc_port"],
            waypoints_file=d["waypoints_file"],
            log_file=d["log_file"],
            state=state,
        )
        for d in DRONES
    ]

    print("=== Connecting to all drones and checking health ===")
    results = await asyncio.gather(
        *[f.connect_and_check_health() for f in followers]
    )

    print("\n=== Connection Summary ===")
    all_ok = True
    for d, ok in zip(DRONES, results):
        status = "SUCCESS" if ok else "FAILED"
        print(f"Drone {d['id']} (port {d['port']}): {status}")
        if not ok:
            all_ok = False

    if not all_ok:
        print("\nOne or more drones failed. Aborting.")
        return

    print("\nAll drones ready. Starting staggered concurrent coverage.\n")

    monitor = asyncio.create_task(collision_monitor(state))

    # Staggered start – prevents the simultaneous arming race
    async def delayed_mission(follower, delay):
        await asyncio.sleep(delay)
        await follower.run_mission()

    await asyncio.gather(
        delayed_mission(followers[0], 0.0),
        delayed_mission(followers[1], STAGGER_DELAY_S),
        delayed_mission(followers[2], STAGGER_DELAY_S * 2),
    )

    monitor.cancel()
    print("\n=== All 3 drones finished their regions ===")


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n[WARN] Interrupted by user.")
