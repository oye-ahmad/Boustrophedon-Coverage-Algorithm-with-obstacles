#!/usr/bin/env python3
"""
Mission executor for the single-agent Boustrophedon coverage mission.

Flow:
1. Briefly spin up rclpy and grab ONE message from /mavros/global_position/global
   to get the drone's home lat/lon (assumes the drone is still on the ground at
   its spawn point when this script starts - see caveat below).
2. Read the local (x, y) waypoints planned by coverage_planner_node from
   ~/coverage_map/waypoints.csv.
3. Convert each local waypoint to (lat, lon) using a flat-earth approximation
   around the home fix (fine at these scales - a few tens of meters).
4. Connect to the SITL drone via MAVSDK, upload the waypoints as a mission,
   arm, start the mission.
5. While the mission runs, poll MAVSDK telemetry.position() and log the
   ACTUAL flown GPS track to ~/coverage_map/actual_path.csv for the
   planned-vs-actual plot (next step).

IMPORTANT CAVEAT (worth noting as one of your documented edge cases):
This assumes the coverage map's local (x, y) frame is aligned with MAVROS's
local ENU frame (x=East, y=North) AND that both share the same origin as the
drone's home position. That's true here because we hand-authored the map
centered on (0,0) to match the drone's spawn point - it would NOT hold if the
map came from SLAM or if the drone were spawned somewhere else in the world.
"""

import asyncio
import csv
import math
import os

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix

from mavsdk import System
from mavsdk.mission import MissionItem, MissionPlan


WAYPOINT_FILE = os.path.expanduser('~/coverage_map/waypoints.csv')
ACTUAL_PATH_FILE = os.path.expanduser('~/coverage_map/actual_path.csv')
MAVSDK_ADDRESS = 'udp://:14540'
MISSION_ALTITUDE_M = 10.0
MISSION_SPEED_M_S = 5.0
EARTH_RADIUS_M = 6378137.0


class HomeFixGrabber(Node):
    """Grabs exactly one NavSatFix from MAVROS then lets the caller stop spinning."""

    def __init__(self):
        super().__init__('home_fix_grabber')
        self.home_lat = None
        self.home_lon = None
        self.create_subscription(
            NavSatFix, '/mavros/global_position/global', self.callback, 10
        )

    def callback(self, msg: NavSatFix):
        if self.home_lat is None:
            self.home_lat = msg.latitude
            self.home_lon = msg.longitude
            self.get_logger().info(f'Home fix: lat={self.home_lat:.7f}, lon={self.home_lon:.7f}')


def get_home_fix(timeout_s=30.0):
    rclpy.init()
    node = HomeFixGrabber()
    start = node.get_clock().now()
    while rclpy.ok() and node.home_lat is None:
        rclpy.spin_once(node, timeout_sec=1.0)
        elapsed = (node.get_clock().now() - start).nanoseconds / 1e9
        if elapsed > timeout_s:
            node.destroy_node()
            rclpy.shutdown()
            raise RuntimeError(
                'Timed out waiting for /mavros/global_position/global. '
                'Is MAVROS running and connected to SITL?'
            )
    lat, lon = node.home_lat, node.home_lon
    node.destroy_node()
    rclpy.shutdown()
    return lat, lon


def local_to_global(x, y, home_lat, home_lon):
    """Flat-earth approximation: x=East offset (m), y=North offset (m)."""
    d_lat = (y / EARTH_RADIUS_M) * (180.0 / math.pi)
    d_lon = (x / (EARTH_RADIUS_M * math.cos(math.radians(home_lat)))) * (180.0 / math.pi)
    return home_lat + d_lat, home_lon + d_lon


def load_local_waypoints(path):
    waypoints = []
    with open(path, newline='') as f:
        reader = csv.DictReader(f)
        for row in reader:
            waypoints.append((float(row['x_local_m']), float(row['y_local_m'])))
    return waypoints


async def run_mission(global_waypoints):
    drone = System()
    await drone.connect(system_address=MAVSDK_ADDRESS)

    print('Waiting for drone connection...')
    async for state in drone.core.connection_state():
        if state.is_connected:
            print('Drone connected.')
            break

    print('Waiting for global position + home lock...')
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print('Position/home lock OK.')
            break

    mission_items = []
    for lat, lon in global_waypoints:
        mission_items.append(
            MissionItem(
                lat, lon, MISSION_ALTITUDE_M,
                MISSION_SPEED_M_S,
                is_fly_through=True,
                gimbal_pitch_deg=float('nan'),
                gimbal_yaw_deg=float('nan'),
                camera_action=MissionItem.CameraAction.NONE,
                loiter_time_s=float('nan'),
                camera_photo_interval_s=float('nan'),
                acceptance_radius_m=float('nan'),
                yaw_deg=float('nan'),
                camera_photo_distance_m=float('nan'),
                vehicle_action=MissionItem.VehicleAction.NONE,
            )
        )

    mission_plan = MissionPlan(mission_items)
    await drone.mission.set_return_to_launch_after_mission(True)

    print('Uploading mission...')
    await drone.mission.upload_mission(mission_plan)

    print('Arming...')
    await drone.action.arm()

    print('Starting mission...')
    await drone.mission.start_mission()

    # Log actual GPS track concurrently until mission completes
    os.makedirs(os.path.dirname(ACTUAL_PATH_FILE), exist_ok=True)
    log_file = open(ACTUAL_PATH_FILE, 'w', newline='')
    writer = csv.writer(log_file)
    writer.writerow(['timestamp_s', 'lat', 'lon', 'alt_m'])

    async def log_position():
        async for position in drone.telemetry.position():
            writer.writerow([
                asyncio.get_event_loop().time(),
                position.latitude_deg,
                position.longitude_deg,
                position.relative_altitude_m,
            ])
            log_file.flush()

    log_task = asyncio.ensure_future(log_position())

    async for progress in drone.mission.mission_progress():
        print(f'Mission progress: {progress.current}/{progress.total}')
        if progress.current == progress.total:
            break

    log_task.cancel()
    log_file.close()
    print(f'Mission complete. Actual GPS track logged to {ACTUAL_PATH_FILE}')


def main():
    print('Fetching home position from MAVROS...')
    home_lat, home_lon = get_home_fix()

    print(f'Loading waypoints from {WAYPOINT_FILE}...')
    local_waypoints = load_local_waypoints(WAYPOINT_FILE)
    print(f'Loaded {len(local_waypoints)} local waypoints.')

    global_waypoints = [
        local_to_global(x, y, home_lat, home_lon) for x, y in local_waypoints
    ]

    asyncio.run(run_mission(global_waypoints))


if __name__ == '__main__':
    main()
