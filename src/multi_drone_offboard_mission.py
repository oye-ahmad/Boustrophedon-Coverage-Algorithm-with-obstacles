#!/usr/bin/env python3
"""
multi_drone_offboard_mission.py

Runs 3 instances of your MAVSDKWaypointFollower concurrently (one per
region), reusing its connect / health-check / arm / takeoff / offboard /
acceptance-radius waypoint-following / land / telemetry-logging logic
UNCHANGED. This file adds:

  - concurrent execution of all 3 drones via asyncio.gather
  - a connection + health check phase for ALL 3 drones BEFORE anything
    else happens (prints SUCCESS/FAILED per drone, aborts if any fails)
  - simple hold-based collision avoidance: a shared background task
    tracks each drone's GLOBAL position (lat/lon) and, if two drones are
    within COLLISION_THRESHOLD_M, makes the higher-id drone hold its
    current setpoint until clear
  - fixes the waypoint-CSV column mismatch: your planner
    (bcd_coverage_node.py) writes `x_local_m, y_local_m, z_m`, while the
    original MAVSDKWaypointFollower._load_waypoints() expected
    `x_local, y_local, z, latitude, longitude`. Those lat/lon columns
    were never actually used in the flight logic (pure local-NED
    offboard control), so this version reads the planner's REAL column
    names directly and drops the unused lat/lon requirement.

Usage:
  python3 multi_drone_offboard_mission.py
(edit DRONES below for your ports / waypoint files / log files)
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
        "id": 1, "port": 14550,
        "grpc_port": 50051,
        "waypoints_file": os.path.expanduser("~/coverage_map/waypoints_drone1.csv"),
        "log_file": os.path.expanduser("~/coverage_map/actual_path_drone1.csv"),
    },
    {
        "id": 2, "port": 14560,
        "grpc_port": 50052,
        "waypoints_file": os.path.expanduser("~/coverage_map/waypoints_drone2.csv"),
        "log_file": os.path.expanduser("~/coverage_map/actual_path_drone2.csv"),
    },
    {
        "id": 3, "port": 14570,
        "grpc_port": 50053,
        "waypoints_file": os.path.expanduser("~/coverage_map/waypoints_drone3.csv"),
        "log_file": os.path.expanduser("~/coverage_map/actual_path_drone3.csv"),
    },
]
ACCEPTANCE_RADIUS_M = 1.2       # same as your original follower
COLLISION_THRESHOLD_M = 5.0
HOLD_CHECK_INTERVAL_S = 1.0
CONNECTION_TIMEOUT_S = 20


def log(drone_id: int, message: str) -> None:
    print(f"Drone {drone_id}: {message}")


class SharedState:
    def __init__(self):
        self.global_positions: Dict[int, Tuple[float, float]] = {}  # id -> (lat, lon)
        self.hold_flags: Dict[int, bool] = {d["id"]: False for d in DRONES}


def haversine_m(lat1, lon1, lat2, lon2) -> float:
    R = 6371000.0
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)
    a = math.sin(dphi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2) ** 2
    return 2 * R * math.asin(math.sqrt(a))


class MultiDroneWaypointFollower:
    """
    Same structure/sequence as your MAVSDKWaypointFollower, extended with
    a drone_id, shared collision-avoidance state, and the corrected CSV
    column names. The connect/health/arm/takeoff/offboard/land/logging
    steps mirror your original implementation as closely as possible.
    """

    def __init__(self, drone_id: int, connection_url: str, waypoints_file: str,
                 log_file: str, state: SharedState):
        self.drone_id = drone_id
        self.connection_url = connection_url
        self.waypoints_file = waypoints_file
        self.log_file = log_file
        self.state = state
        self.drone = System()
        self.is_logging = False

    # ------------------------------------------------------------------
    async def connect_and_check_health(self) -> bool:
        log(self.drone_id, f"Connecting on {self.connection_url}...")
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

        log(self.drone_id, "Waiting for health and global position lock...")
        try:
            async def _wait_health():
                async for health in self.drone.telemetry.health():
                    if health.is_global_position_ok and health.is_home_position_ok:
                        return True

            await asyncio.wait_for(_wait_health(), timeout=CONNECTION_TIMEOUT_S)
            log(self.drone_id, "Health checks passed: Global position lock acquired.")
            return True
        except asyncio.TimeoutError:
            log(self.drone_id, "FAILED to acquire global position lock in time.")
            return False

    # ------------------------------------------------------------------
    def _load_waypoints(self, filepath: str):
        """Reads the ACTUAL columns bcd_coverage_multi_node.py writes:
        seq, x_local_m, y_local_m, z_m (no lat/lon - unused in flight)."""
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

    # ------------------------------------------------------------------
    async def run_mission(self):
        waypoints = self._load_waypoints(self.waypoints_file)
        if not waypoints:
            log(self.drone_id, "No valid waypoints found. Skipping mission.")
            return

        self.is_logging = True
        logger_task = asyncio.create_task(self._log_telemetry())
        global_tracker_task = asyncio.create_task(self._track_global_position())

        try:
            log(self.drone_id, "Arming...")
            await self.drone.action.arm()

            log(self.drone_id, "Taking off...")
            await self.drone.action.takeoff()
            await asyncio.sleep(8)

            initial_target = waypoints[0]
            setpoint = PositionNedYaw(
                north_m=initial_target["x_local"],
                east_m=initial_target["y_local"],
                down_m=-initial_target["z"],
                yaw_deg=0.0,
            )
            await self.drone.offboard.set_position_ned(setpoint)

            log(self.drone_id, "Switching to OFFBOARD control mode...")
            try:
                await self.drone.offboard.start()
            except OffboardError as e:
                log(self.drone_id, f"Offboard start failed: {e._result.result}. Landing.")
                await self.drone.action.land()
                return

            log(self.drone_id, f"Starting coverage execution across {len(waypoints)} waypoints...")

            for idx, wp in enumerate(waypoints):
                # Respect collision-avoidance hold before committing to the
                # next waypoint's setpoint.
                while self.state.hold_flags.get(self.drone_id, False):
                    await asyncio.sleep(HOLD_CHECK_INTERVAL_S)

                target_ned = PositionNedYaw(
                    north_m=wp["x_local"],
                    east_m=wp["y_local"],
                    down_m=-wp["z"],
                    yaw_deg=0.0,
                )
                await self.drone.offboard.set_position_ned(target_ned)

                while True:
                    if self.state.hold_flags.get(self.drone_id, False):
                        await asyncio.sleep(HOLD_CHECK_INTERVAL_S)
                        continue

                    async for ned_pos in self.drone.telemetry.position_velocity_ned():
                        curr_north = ned_pos.position.north_m
                        curr_east = ned_pos.position.east_m
                        curr_down = ned_pos.position.down_m

                        dist = math.sqrt(
                            (curr_north - wp["x_local"]) ** 2 +
                            (curr_east - wp["y_local"]) ** 2 +
                            (curr_down - (-wp["z"])) ** 2
                        )
                        break

                    if dist <= ACCEPTANCE_RADIUS_M:
                        log(self.drone_id, f"Reached Waypoint {idx + 1}/{len(waypoints)} (Dist: {dist:.2f}m)")
                        break
                    await asyncio.sleep(0.1)

            log(self.drone_id, "All waypoints successfully executed!")

            log(self.drone_id, "Stopping Offboard mode and commanding Land...")
            try:
                await self.drone.offboard.stop()
            except OffboardError as e:
                log(self.drone_id, f"WARN: Failed to stop offboard cleanly: {e._result.result}")

            await self.drone.action.land()

            async for is_armed in self.drone.telemetry.armed():
                if not is_armed:
                    log(self.drone_id, "Disarmed. Region coverage mission accomplished.")
                    break
                await asyncio.sleep(1)

        finally:
            self.is_logging = False
            await logger_task
            global_tracker_task.cancel()

    # ------------------------------------------------------------------
    async def _log_telemetry(self):
        """Same as your original _log_telemetry, per-drone log file."""
        os.makedirs(os.path.dirname(self.log_file), exist_ok=True)
        with open(self.log_file, mode="w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["timestamp", "latitude", "longitude", "abs_altitude", "rel_altitude"])

            while self.is_logging:
                async for pos in self.drone.telemetry.position():
                    timestamp = time.time()
                    writer.writerow([
                        f"{timestamp:.3f}",
                        f"{pos.latitude_deg:.8f}",
                        f"{pos.longitude_deg:.8f}",
                        f"{pos.absolute_altitude_m:.2f}",
                        f"{pos.relative_altitude_m:.2f}",
                    ])
                    f.flush()
                    break
                await asyncio.sleep(0.5)

    async def _track_global_position(self):
        """Feeds SharedState.global_positions for the collision monitor."""
        async for pos in self.drone.telemetry.position():
            self.state.global_positions[self.drone_id] = (pos.latitude_deg, pos.longitude_deg)


async def collision_monitor(state: SharedState):
    """Pairwise distance check every second; higher-id drone holds within 5 m."""
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
            connection_url=f"udp://:{d['port']}",
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

    print("\nAll drones ready. Starting concurrent region coverage with collision avoidance.\n")

    monitor_task = asyncio.create_task(collision_monitor(state))
    await asyncio.gather(*[f.run_mission() for f in followers])
    monitor_task.cancel()

    print("\n=== All 3 drones have completed their coverage regions ===")


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("[WARN] Script interrupted by user.")
