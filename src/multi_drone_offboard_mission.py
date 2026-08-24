#!/usr/bin/env python3
"""
multi_drone_offboard_mission.py
"""

import os
import csv
import time
import math
import asyncio
from typing import Dict, List, Tuple

from mavsdk import System
from mavsdk.offboard import OffboardError, PositionNedYaw


# ---- Configuration -----------------------------------------------------
DRONES = [
    {
        "id": 1, "port": 14550, "grpc_port": 50051,
        "waypoints_file": os.path.expanduser("~/coverage_map/waypoints_drone1.csv"),
        "log_file": os.path.expanduser("~/coverage_map/actual_path_drone1.csv"),
    },
    {
        "id": 2, "port": 14560, "grpc_port": 50052,
        "waypoints_file": os.path.expanduser("~/coverage_map/waypoints_drone2.csv"),
        "log_file": os.path.expanduser("~/coverage_map/actual_path_drone2.csv"),
    },
    {
        "id": 3, "port": 14570, "grpc_port": 50053,
        "waypoints_file": os.path.expanduser("~/coverage_map/waypoints_drone3.csv"),
        "log_file": os.path.expanduser("~/coverage_map/actual_path_drone3.csv"),
    },
]
ACCEPTANCE_RADIUS_M = 1.2
COLLISION_THRESHOLD_M = 5.0
HOLD_CHECK_INTERVAL_S = 0.2
CONNECTION_TIMEOUT_S = 30
TAKEOFF_ALTITUDE_M = 2.5


def log(drone_id: int, message: str) -> None:
    print(f"Drone {drone_id}: {message}")


class SharedState:
    def __init__(self):
        self.global_positions: Dict[int, Tuple[float, float]] = {}  # id -> (lat, lon)
        self.hold_flags: Dict[int, bool] = {d["id"]: False for d in DRONES}


def haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    R = 6371000.0
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)
    a = math.sin(dphi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2) ** 2
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

        # Live telemetry cached values to avoid opening/closing gRPC streams
        self.current_rel_alt = 0.0
        self.current_ned_pos = (0.0, 0.0, 0.0)

    async def connect_and_check_health(self) -> bool:
        log(self.drone_id, f"Connecting on {self.connection_url} (gRPC port {self.grpc_port})...")
        await self.drone.connect(system_address=self.connection_url)

        try:
            async def _wait_connected():
                async for state in self.drone.core.connection_state():
                    if state.is_connected:
                        return True

            await asyncio.wait_for(_wait_connected(), timeout=CONNECTION_TIMEOUT_S)
            log(self.drone_id, "Connection SUCCESSFUL.")
        except asyncio.TimeoutError:
            log(self.drone_id, f"Connection FAILED (no heartbeat within {CONNECTION_TIMEOUT_S}s).")
            return False

        log(self.drone_id, "Waiting for position lock and armable health state...")
        try:
            async def _wait_health():
                async for health in self.drone.telemetry.health():
                    pos_ok = health.is_global_position_ok or health.is_local_position_ok
                    if pos_ok and health.is_home_position_ok and health.is_armable:
                        return True
                    await asyncio.sleep(1)

            await asyncio.wait_for(_wait_health(), timeout=CONNECTION_TIMEOUT_S)
            log(self.drone_id, "Health checks passed: Position lock & Home OK.")
            return True
        except asyncio.TimeoutError:
            log(self.drone_id, "FAILED to acquire required health checks in time.")
            return False

    def _load_waypoints(self, filepath: str):
        waypoints = []
        if not os.path.exists(filepath):
            log(self.drone_id, f"ERROR: waypoint CSV not found at {filepath}")
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

    async def run_mission(self):
        waypoints = self._load_waypoints(self.waypoints_file)
        if not waypoints:
            log(self.drone_id, "No valid waypoints found. Skipping mission.")
            return

        self.is_logging = True
        
        # Continuous long-running telemetry background tasks
        logger_task = asyncio.create_task(self._log_telemetry())
        pos_tracker_task = asyncio.create_task(self._track_positions())

        try:
            # Step 1: Set takeoff altitude & Arm
            log(self.drone_id, f"Setting takeoff altitude to {TAKEOFF_ALTITUDE_M}m...")
            await self.drone.action.set_takeoff_altitude(TAKEOFF_ALTITUDE_M)

            log(self.drone_id, "Arming drone...")
            try:
                await self.drone.action.arm()
            except Exception as e:
                log(self.drone_id, f"Arming failed: {e}")
                return

            # Step 2: Native Takeoff
            log(self.drone_id, "Taking off via action.takeoff()...")
            try:
                await self.drone.action.takeoff()
            except Exception as e:
                log(self.drone_id, f"Takeoff command failed: {e}")
                return

            # Step 3: Wait for Takeoff Altitude using cached telemetry
            log(self.drone_id, "Climbing to takeoff altitude...")
            while True:
                if self.current_rel_alt >= (TAKEOFF_ALTITUDE_M - 0.4):
                    log(self.drone_id, f"Airborne! Current Alt: {self.current_rel_alt:.2f}m")
                    break
                await asyncio.sleep(0.3)

            # Step 4: Stream initial setpoint while airborne
            takeoff_setpoint = PositionNedYaw(
                north_m=0.0,
                east_m=0.0,
                down_m=-TAKEOFF_ALTITUDE_M,
                yaw_deg=0.0,
            )
            log(self.drone_id, "Streaming setpoints in mid-air for Offboard mode...")
            for _ in range(15):
                await self.drone.offboard.set_position_ned(takeoff_setpoint)
                await asyncio.sleep(0.1)

            # Step 5: Switch to OFFBOARD mode
            log(self.drone_id, "Switching to OFFBOARD mode...")
            try:
                await self.drone.offboard.start()
                log(self.drone_id, "OFFBOARD mode active!")
            except OffboardError as e:
                log(self.drone_id, f"Offboard start ERROR: {e._result.result}")
                return

            # Step 6: Execute Waypoints
            log(self.drone_id, f"Executing {len(waypoints)} waypoints...")
            current_target = takeoff_setpoint

            for idx, wp in enumerate(waypoints):
                target_ned = PositionNedYaw(
                    north_m=wp["x_local"],
                    east_m=wp["y_local"],
                    down_m=-wp["z"],
                    yaw_deg=0.0,
                )

                while True:
                    if self.state.hold_flags.get(self.drone_id, False):
                        await self.drone.offboard.set_position_ned(current_target)
                        await asyncio.sleep(HOLD_CHECK_INTERVAL_S)
                        continue

                    current_target = target_ned
                    await self.drone.offboard.set_position_ned(current_target)

                    curr_north, curr_east, curr_down = self.current_ned_pos

                    dist = math.sqrt(
                        (curr_north - wp["x_local"]) ** 2 +
                        (curr_east - wp["y_local"]) ** 2 +
                        (curr_down - (-wp["z"])) ** 2
                    )

                    if dist <= ACCEPTANCE_RADIUS_M:
                        log(self.drone_id, f"Reached Waypoint {idx + 1}/{len(waypoints)} (Dist: {dist:.2f}m)")
                        break

                    await asyncio.sleep(0.1)

            log(self.drone_id, "All waypoints completed successfully!")

            log(self.drone_id, "Stopping Offboard mode and commanding Land...")
            try:
                await self.drone.offboard.stop()
            except OffboardError as e:
                log(self.drone_id, f"WARN: Failed to stop offboard cleanly: {e._result.result}")

            await self.drone.action.land()

            async for is_armed in self.drone.telemetry.armed():
                if not is_armed:
                    log(self.drone_id, "Disarmed and landed successfully.")
                    break
                await asyncio.sleep(1)

        finally:
            self.is_logging = False
            logger_task.cancel()
            pos_tracker_task.cancel()

    async def _log_telemetry(self):
        os.makedirs(os.path.dirname(self.log_file), exist_ok=True)
        with open(self.log_file, mode="w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["timestamp", "latitude", "longitude", "abs_altitude", "rel_altitude"])

            try:
                async for pos in self.drone.telemetry.position():
                    if not self.is_logging:
                        break
                    timestamp = time.time()
                    writer.writerow([
                        f"{timestamp:.3f}",
                        f"{pos.latitude_deg:.8f}",
                        f"{pos.longitude_deg:.8f}",
                        f"{pos.absolute_altitude_m:.2f}",
                        f"{pos.relative_altitude_m:.2f}",
                    ])
                    f.flush()
                    await asyncio.sleep(0.5)
            except asyncio.CancelledError:
                pass

    async def _track_positions(self):
        """Single long-lived task that listens to telemetry streams continuously."""
        async def _track_global():
            try:
                async for pos in self.drone.telemetry.position():
                    self.state.global_positions[self.drone_id] = (pos.latitude_deg, pos.longitude_deg)
                    self.current_rel_alt = pos.relative_altitude_m
            except asyncio.CancelledError:
                pass

        async def _track_ned():
            try:
                async for ned in self.drone.telemetry.position_velocity_ned():
                    self.current_ned_pos = (
                        ned.position.north_m,
                        ned.position.east_m,
                        ned.position.down_m
                    )
            except asyncio.CancelledError:
                pass

        await asyncio.gather(_track_global(), _track_ned())


async def collision_monitor(state: SharedState):
    drone_ids = [d["id"] for d in DRONES]
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
                lower_id, higher_id = min(id_a, id_b), max(id_a, id_b)

                if dist < COLLISION_THRESHOLD_M:
                    if not state.hold_flags[higher_id]:
                        log(higher_id, f"WITHIN {dist:.1f} m of Drone {lower_id} - HOLDING.")
                    state.hold_flags[higher_id] = True
                else:
                    if state.hold_flags[higher_id]:
                        log(higher_id, f"Clear of Drone {lower_id} ({dist:.1f} m) - RESUMING.")
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
    connect_results = await asyncio.gather(*[f.connect_and_check_health() for f in followers])

    print("\n=== Connection Summary ===")
    all_ok = True
    for d, ok in zip(DRONES, connect_results):
        print(f"Drone {d['id']} (port {d['port']}): {'SUCCESS' if ok else 'FAILED'}")
        if not ok:
            all_ok = False

    if not all_ok:
        print("\nOne or more drones failed to connect/health-check. Aborting mission.")
        return

    print("\nAll drones ready. Starting concurrent takeoff and mission...\n")

    monitor_task = asyncio.create_task(collision_monitor(state))
    await asyncio.gather(*[f.run_mission() for f in followers])
    monitor_task.cancel()

    print("\n=== All drones completed their missions ===")


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("[WARN] Script interrupted by user.")
