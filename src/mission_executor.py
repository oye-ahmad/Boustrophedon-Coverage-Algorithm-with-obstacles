#!/usr/bin/env python3
"""
Improved Mission Executor for single-agent Boustrophedon coverage.

Changes vs previous version:
- Proper takeoff sequence (Action plugin)
- Explicit switch to AUTO mode (works on ArduCopter)
- Better waiting / health checks
- Parameters at the top (easy to change)
- Clearer logging
- Still logs actual GPS track for the planned-vs-actual plot
"""

import asyncio
import csv
import math
import os
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix

from mavsdk import System
from mavsdk.mission_raw import MissionItem
from mavsdk.mavlink_direct import MavlinkMessage


# ======================== CONFIG ========================
WAYPOINT_FILE     = os.path.expanduser('~/coverage_map/waypoints.csv')
ACTUAL_PATH_FILE  = os.path.expanduser('~/coverage_map/actual_path.csv')
MAVSDK_ADDRESS    = 'udp://:14540'          # SITL default
MISSION_ALTITUDE  = 10.0                    # meters (relative)
TAKEOFF_ALTITUDE  = 10.0                    # meters
ACCEPTANCE_RADIUS = 2.0                     # meters
EARTH_RADIUS_M    = 6378137.0
# ========================================================


class HomeFixGrabber(Node):
    """Grab one NavSatFix from MAVROS then stop."""

    def __init__(self):
        super().__init__('home_fix_grabber')
        self.home_lat = None
        self.home_lon = None
        self.create_subscription(
            NavSatFix,
            '/mavros/global_position/global',
            self.callback,
            qos_profile_sensor_data
        )

    def callback(self, msg: NavSatFix):
        if self.home_lat is None:
            self.home_lat = msg.latitude
            self.home_lon = msg.longitude
            self.get_logger().info(
                f'Home fix received → lat={self.home_lat:.7f}, lon={self.home_lon:.7f}'
            )


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
                'Timeout waiting for /mavros/global_position/global.\n'
                'Make sure MAVROS is running and connected to SITL.'
            )

    lat, lon = node.home_lat, node.home_lon
    node.destroy_node()
    rclpy.shutdown()
    return lat, lon


def local_to_global(x, y, home_lat, home_lon):
    """Flat-earth: x = East (m), y = North (m)"""
    d_lat = (y / EARTH_RADIUS_M) * (180.0 / math.pi)
    d_lon = (x / (EARTH_RADIUS_M * math.cos(math.radians(home_lat)))) * (180.0 / math.pi)
    return home_lat + d_lat, home_lon + d_lon


def load_local_waypoints(path):
    waypoints = []
    with open(path, newline='') as f:
        reader = csv.DictReader(f)
        for row in reader:
            x = float(row['x_local_m'])
            y = float(row['y_local_m'])
            # z is optional (newer planner writes it)
            z = float(row.get('z_m', MISSION_ALTITUDE))
            waypoints.append((x, y, z))
    return waypoints


async def set_mode_auto(drone: System):
    """Force ArduCopter into AUTO mode via COMMAND_LONG."""
    # MAV_CMD_DO_SET_MODE = 176
    # custom_mode = 3  →  AUTO for ArduCopter
    fields = (
        '{"target_system":1,"target_component":1,"command":176,'
        '"confirmation":0,"param1":1,"param2":3,'
        '"param3":0,"param4":0,"param5":0,"param6":0,"param7":0}'
    )
    msg = MavlinkMessage(
        message_name='COMMAND_LONG',
        system_id=245,
        component_id=190,
        target_system_id=1,
        target_component_id=1,
        fields_json=fields,
    )
    await drone.mavlink_direct.send_message(msg)
    print('→ Sent DO_SET_MODE (AUTO)')


async def run_mission(global_waypoints):
    drone = System()
    await drone.connect(system_address=MAVSDK_ADDRESS)

    print('Waiting for MAVSDK connection...')
    async for state in drone.core.connection_state():
        if state.is_connected:
            print('✓ Drone connected')
            break

    print('Waiting for position + home lock...')
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print('✓ Position & home OK')
            break

    # ---------- Build mission ----------
    MAV_FRAME_GLOBAL_RELATIVE_ALT = 3
    MAV_CMD_NAV_TAKEOFF          = 22
    MAV_CMD_NAV_WAYPOINT         = 16
    MAV_CMD_NAV_RETURN_TO_LAUNCH = 20
    MAV_MISSION_TYPE_MISSION     = 0

    mission_items = []

    # 1. Explicit TAKEOFF item (required by ArduCopter)
    first_lat, first_lon, first_alt = global_waypoints[0]
    mission_items.append(
        MissionItem(
            0,                                      # seq
            MAV_FRAME_GLOBAL_RELATIVE_ALT,
            MAV_CMD_NAV_TAKEOFF,
            1,                                      # current
            1,                                      # autocontinue
            0.0, 0.0, 0.0, float('nan'),
            int(first_lat * 1e7),
            int(first_lon * 1e7),
            float(TAKEOFF_ALTITUDE),
            MAV_MISSION_TYPE_MISSION,
        )
    )

    # 2. Coverage waypoints
    for i, (lat, lon, alt) in enumerate(global_waypoints):
        mission_items.append(
            MissionItem(
                i + 1,
                MAV_FRAME_GLOBAL_RELATIVE_ALT,
                MAV_CMD_NAV_WAYPOINT,
                0, 1,
                0.0,                                # hold time
                ACCEPTANCE_RADIUS,                  # acceptance radius
                0.0,                                # pass-through radius
                float('nan'),                       # yaw
                int(lat * 1e7),
                int(lon * 1e7),
                float(alt),
                MAV_MISSION_TYPE_MISSION,
            )
        )

    # 3. RTL at the end
    mission_items.append(
        MissionItem(
            len(mission_items),
            MAV_FRAME_GLOBAL_RELATIVE_ALT,
            MAV_CMD_NAV_RETURN_TO_LAUNCH,
            0, 1,
            0.0, 0.0, 0.0, 0.0,
            0, 0, 0.0,
            MAV_MISSION_TYPE_MISSION,
        )
    )

    print(f'Uploading mission ({len(mission_items)} items)...')
    await drone.mission_raw.clear_mission()
    await drone.mission_raw.upload_mission(mission_items)
    print('✓ Mission uploaded')

    # ---------- Arm + Takeoff + AUTO ----------
    print('Arming...')
    await drone.action.arm()
    await asyncio.sleep(1.0)

    print(f'Taking off to {TAKEOFF_ALTITUDE} m...')
    await drone.action.set_takeoff_altitude(TAKEOFF_ALTITUDE)
    await drone.action.takeoff()

    # Wait until we are roughly at takeoff altitude
    print('Waiting to reach takeoff altitude...')
    async for position in drone.telemetry.position():
        if position.relative_altitude_m >= TAKEOFF_ALTITUDE * 0.90:
            print(f'✓ Reached {position.relative_altitude_m:.1f} m')
            break

    await asyncio.sleep(1.0)

    print('Switching to AUTO mode...')
    await set_mode_auto(drone)
    await asyncio.sleep(1.5)

    print('Starting mission...')
    try:
        await drone.mission_raw.start_mission()
    except Exception as e:
        # Sometimes AUTO already started it – not fatal
        print(f'Note from start_mission(): {e}')

    # ---------- Log actual path ----------
    os.makedirs(os.path.dirname(ACTUAL_PATH_FILE), exist_ok=True)
    log_file = open(ACTUAL_PATH_FILE, 'w', newline='')
    writer = csv.writer(log_file)
    writer.writerow(['timestamp_s', 'lat', 'lon', 'alt_m'])

    async def log_position():
        async for pos in drone.telemetry.position():
            writer.writerow([
                asyncio.get_event_loop().time(),
                pos.latitude_deg,
                pos.longitude_deg,
                pos.relative_altitude_m,
            ])
            log_file.flush()

    log_task = asyncio.ensure_future(log_position())

    print('Mission running – logging GPS track...')
    try:
        async for progress in drone.mission_raw.mission_progress():
            print(f'  Progress: {progress.current} / {progress.total}')
            if progress.current >= progress.total:
                break
    except Exception as e:
        print(f'Mission progress ended: {e}')

    log_task.cancel()
    log_file.close()
    print(f'\n✓ Mission finished')
    print(f'  Actual track saved → {ACTUAL_PATH_FILE}')


def main():
    print('=== Boustrophedon Mission Executor ===')
    print('1. Fetching home position from MAVROS...')
    try:
        home_lat, home_lon = get_home_fix()
    except Exception as e:
        print(f'ERROR: {e}')
        sys.exit(1)

    print(f'2. Loading waypoints from {WAYPOINT_FILE}')
    if not os.path.exists(WAYPOINT_FILE):
        print('ERROR: Waypoint file not found. Run the coverage planner first.')
        sys.exit(1)

    local_wps = load_local_waypoints(WAYPOINT_FILE)
    print(f'   Loaded {len(local_wps)} waypoints')

    global_wps = [
        (*local_to_global(x, y, home_lat, home_lon), z)
        for x, y, z in local_wps
    ]

    print('3. Starting mission sequence...')
    asyncio.run(run_mission(global_wps))


if __name__ == '__main__':
    main()
