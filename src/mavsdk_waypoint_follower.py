#!/usr/bin/env python3
import os
import sys
import csv
import time
import asyncio
import argparse
import numpy as np
from mavsdk import System
from mavsdk.offboard import OffboardError, PositionNedYaw


class MAVSDKWaypointFollower:
    def __init__(self, connection_url: str, waypoints_file: str, log_file: str):
        self.connection_url = connection_url
        self.waypoints_file = waypoints_file
        self.log_file = log_file
        self.drone = System()
        self.is_logging = False

    async def run(self):
        """Main execution sequence."""
        # 1. Connect to Drone
        print(f"[INFO] Connecting to drone on {self.connection_url}...")
        await self.drone.connect(system_address=self.connection_url)

        async for state in self.drone.core.connection_state():
            if state.is_connected:
                print("[INFO] Drone connected successfully!")
                break

        # 2. Check Health & Global Position Estimate
        print("[INFO] Waiting for drone health and global position lock...")
        async for health in self.drone.telemetry.health():
            if health.is_global_position_ok and health.is_home_position_ok:
                print("[INFO] Health checks passed: Global position lock acquired.")
                break

        # 3. Read Waypoints from CSV
        waypoints = self._load_waypoints(self.waypoints_file)
        if not waypoints:
            print("[ERROR] No valid waypoints found. Exiting.")
            return

        # Start Telemetry Logging Task
        self.is_logging = True
        logger_task = asyncio.create_task(self._log_telemetry())

        try:
            # 4. Arm and Take Off
            print("[INFO] Arming drone...")
            await self.drone.action.arm()

            print("[INFO] Taking off...")
            await self.drone.action.takeoff()
            await asyncio.sleep(8)  # Wait for takeoff stabilization

            # 5. Set Initial Target Pose for Offboard Mode
            initial_target = waypoints[0]
            # PX4 NED Frame: North = local_x, East = local_y, Down = -local_z
            setpoint = PositionNedYaw(
                north_m=initial_target['x_local'],
                east_m=initial_target['y_local'],
                down_m=-initial_target['z'],
                yaw_deg=0.0
            )
            await self.drone.offboard.set_position_ned(setpoint)

            print("[INFO] Switching to OFFBOARD control mode...")
            try:
                await self.drone.offboard.start()
            except OffboardError as e:
                print(f"[ERROR] Offboard start failed: {e._result.result}. Landing.")
                await self.drone.action.land()
                return

            # 6. Execute Waypoint Trajectory (Lookahead / Acceptance Radius Loop)
            print(f"[INFO] Starting coverage execution across {len(waypoints)} waypoints...")
            acceptance_radius_m = 1.2  # Reach distance threshold

            for idx, wp in enumerate(waypoints):
                target_ned = PositionNedYaw(
                    north_m=wp['x_local'],
                    east_m=wp['y_local'],
                    down_m=-wp['z'],
                    yaw_deg=0.0
                )
                await self.drone.offboard.set_position_ned(target_ned)

                # Wait until drone enters acceptance radius of current waypoint
                while True:
                    async for ned_pos in self.drone.telemetry.position_velocity_ned():
                        curr_north = ned_pos.position.north_m
                        curr_east = ned_pos.position.east_m
                        curr_down = ned_pos.position.down_m

                        dist = np.sqrt(
                            (curr_north - wp['x_local']) ** 2 +
                            (curr_east - wp['y_local']) ** 2 +
                            (curr_down - (-wp['z'])) ** 2
                        )

                        if dist <= acceptance_radius_m:
                            print(f"[INFO] Reached Waypoint {idx + 1}/{len(waypoints)} (Dist: {dist:.2f}m)")
                            break
                        
                        await asyncio.sleep(0.1)
                        break  # Break telemetry generator step to loop

                    if dist <= acceptance_radius_m:
                        break

            print("[INFO] All waypoints successfully executed!")

            # 7. Stop Offboard Mode and Land
            print("[INFO] Stopping Offboard mode and commanding Land...")
            try:
                await self.drone.offboard.stop()
            except OffboardError as e:
                print(f"[WARN] Failed to stop offboard cleanly: {e._result.result}")

            await self.drone.action.land()

            # Wait for landing disarm
            async for is_armed in self.drone.telemetry.armed():
                if not is_armed:
                    print("[INFO] Drone disarmed. Mission accomplished.")
                    break
                await asyncio.sleep(1)

        finally:
            self.is_logging = False
            await logger_task

    def _load_waypoints(self, filepath: str):
        waypoints = []
        if not os.path.exists(filepath):
            print(f"[ERROR] Waypoint CSV not found at: {filepath}")
            return waypoints

        with open(filepath, mode='r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                waypoints.append({
                    'seq': int(row['seq']),
                    'x_local': float(row['x_local']),
                    'y_local': float(row['y_local']),
                    'z': float(row['z']),
                    'latitude': float(row['latitude']),
                    'longitude': float(row['longitude'])
                })
        return waypoints

    async def _log_telemetry(self):
        """Background asynchronous task fetching real-time telemetry at 2Hz."""
        os.makedirs(os.path.dirname(self.log_file), exist_ok=True)
        
        with open(self.log_file, mode='w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['timestamp', 'latitude', 'longitude', 'abs_altitude', 'rel_altitude'])

            while self.is_logging:
                async for pos in self.drone.telemetry.position():
                    timestamp = time.time()
                    writer.writerow([
                        f"{timestamp:.3f}",
                        f"{pos.latitude_deg:.8f}",
                        f"{pos.longitude_deg:.8f}",
                        f"{pos.absolute_altitude_m:.2f}",
                        f"{pos.relative_altitude_m:.2f}"
                    ])
                    f.flush()
                    break
                await asyncio.sleep(0.5)  # 2 Hz logging rate


# ==============================================================================
# Main Entry Point
# ==============================================================================

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="MAVSDK Offboard Waypoint Follower for BCD Coverage Path")
    parser.add_argument('--url', type=str, default="udp://:14540", help="MAVSDK connection URL (e.g. serial:///dev/ttyACM0:57600 or udp://:14540)")
    parser.add_argument('--waypoints', type=str, default=os.path.expanduser('~/coverage_planner/waypoints.csv'), help="Path to input waypoints.csv")
    parser.add_argument('--logfile', type=str, default=os.path.expanduser('~/coverage_planner/coverage_track.csv'), help="Path to output telemetry log")

    args = parser.parse_args()

    follower = MAVSDKWaypointFollower(
        connection_url=args.url,
        waypoints_file=args.waypoints,
        log_file=args.logfile
    )

    try:
        asyncio.run(follower.run())
    except KeyboardInterrupt:
        print("[WARN] Script interrupted by user.")
