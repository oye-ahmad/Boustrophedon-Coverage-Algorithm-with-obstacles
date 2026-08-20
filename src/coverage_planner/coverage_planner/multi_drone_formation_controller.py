#!/usr/bin/env python3

import math
import time
import threading

import rclpy
from rclpy.node import Node

from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy,
    DurabilityPolicy
)

from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL


class MultiDroneFormation(Node):

    def __init__(self):

        super().__init__('multi_drone_formation_controller')

        # ============================================================
        # CONFIGURATION
        # ============================================================

        self.drone_names = [
            'drone1',
            'drone2',
            'drone3'
        ]

        # ------------------------------------------------------------
        # FLIGHT PARAMETERS
        # ------------------------------------------------------------

        self.takeoff_altitude = 5.0

        self.forward_speed = 1.0

        self.forward_distance = 15.0

        self.hold_time = 10.0

        # ------------------------------------------------------------
        # COLLISION AVOIDANCE
        # ------------------------------------------------------------

        # Hard minimum separation.
        self.min_distance = 5.0

        # Start avoiding before reaching minimum distance.
        self.avoidance_distance = 7.0

        # Emergency distance.
        self.emergency_distance = 4.5

        self.max_avoidance_speed = 1.5

        self.max_velocity = 2.0

        # ------------------------------------------------------------
        # POSITION PARAMETERS
        # ------------------------------------------------------------

        self.position_tolerance = 0.5

        # ------------------------------------------------------------
        # CONTROL
        # ------------------------------------------------------------

        self.control_rate = 20.0

        # ============================================================
        # STATE
        # ============================================================

        self.states = {}

        # Local position
        self.positions = {}

        # GPS position
        self.global_positions = {}

        # Reference GPS position
        self.reference_lat = None
        self.reference_lon = None

        self.home_positions = {}

        self.armed = {}
        self.connected = {}

        self.velocity_publishers = {}

        self.arm_clients = {}
        self.mode_clients = {}
        self.takeoff_clients = {}
        self.land_clients = {}

        self.lock = threading.Lock()

        self.mission_started = False
        self.mission_finished = False

        self.stop_requested = False

        # ============================================================
        # MAVROS QoS
        # ============================================================

        # IMPORTANT:
        #
        # MAVROS position publishers commonly use BEST_EFFORT.
        #
        # A default ROS2 subscription is RELIABLE.
        #
        # RELIABLE subscriber + BEST_EFFORT publisher
        # causes:
        #
        # "offering incompatible QoS"
        #
        # Therefore explicitly use BEST_EFFORT.

        mavros_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )

        state_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )

        # ============================================================
        # CREATE ROS INTERFACES
        # ============================================================

        for drone in self.drone_names:

            self.states[drone] = State()

            self.positions[drone] = None

            self.global_positions[drone] = None

            self.home_positions[drone] = None

            self.armed[drone] = False

            self.connected[drone] = False

            # --------------------------------------------------------
            # STATE
            # --------------------------------------------------------

            self.create_subscription(
                State,
                f'/{drone}/state',
                lambda msg, d=drone:
                self.state_callback(msg, d),
                state_qos
            )

            # --------------------------------------------------------
            # LOCAL POSITION
            # --------------------------------------------------------

            self.create_subscription(
                PoseStamped,
                f'/{drone}/local_position/pose',
                lambda msg, d=drone:
                self.position_callback(msg, d),
                mavros_qos
            )

            # --------------------------------------------------------
            # GLOBAL GPS POSITION
            # --------------------------------------------------------

            self.create_subscription(
                NavSatFix,
                f'/{drone}/global_position/global',
                lambda msg, d=drone:
                self.global_position_callback(msg, d),
                mavros_qos
            )

            # --------------------------------------------------------
            # VELOCITY COMMAND
            # --------------------------------------------------------

            self.velocity_publishers[drone] = self.create_publisher(
                TwistStamped,
                f'/{drone}/setpoint_velocity/cmd_vel',
                10
            )

            # --------------------------------------------------------
            # ARM
            # --------------------------------------------------------

            self.arm_clients[drone] = self.create_client(
                CommandBool,
                f'/{drone}/cmd/arming'
            )

            # --------------------------------------------------------
            # MODE
            # --------------------------------------------------------

            self.mode_clients[drone] = self.create_client(
                SetMode,
                f'/{drone}/cmd/set_mode'
            )

            # --------------------------------------------------------
            # TAKEOFF
            # --------------------------------------------------------

            self.takeoff_clients[drone] = self.create_client(
                CommandTOL,
                f'/{drone}/cmd/takeoff'
            )

            # --------------------------------------------------------
            # LAND
            # --------------------------------------------------------

            self.land_clients[drone] = self.create_client(
                CommandTOL,
                f'/{drone}/cmd/land'
            )

        # ============================================================
        # CONTROL TIMER
        # ============================================================

        self.timer = self.create_timer(
            1.0 / self.control_rate,
            self.control_loop
        )

        # ============================================================
        # LOGGING
        # ============================================================

        self.get_logger().info(
            '===================================================='
        )

        self.get_logger().info(
            '     MULTI-DRONE FORMATION CONTROLLER'
        )

        self.get_logger().info(
            '===================================================='
        )

        self.get_logger().info(
            'Drones: drone1, drone2, drone3'
        )

        self.get_logger().info(
            f'Takeoff altitude: '
            f'{self.takeoff_altitude:.1f} m'
        )

        self.get_logger().info(
            f'Forward speed: '
            f'{self.forward_speed:.1f} m/s'
        )

        self.get_logger().info(
            f'Forward distance: '
            f'{self.forward_distance:.1f} m'
        )

        self.get_logger().info(
            f'Minimum separation: '
            f'{self.min_distance:.1f} m'
        )

        self.get_logger().info(
            f'Avoidance starts: '
            f'{self.avoidance_distance:.1f} m'
        )

        self.get_logger().info(
            f'Emergency distance: '
            f'{self.emergency_distance:.1f} m'
        )

        self.get_logger().info(
            f'Hold time: '
            f'{self.hold_time:.1f} seconds'
        )

        self.get_logger().info(
            'Using GPS-based common coordinate system '
            'for collision avoidance.'
        )

        self.get_logger().info(
            'Waiting for all drones...'
        )

    # ================================================================
    # CALLBACKS
    # ================================================================

    def state_callback(self, msg, drone):

        with self.lock:

            self.states[drone] = msg

            self.connected[drone] = msg.connected

            self.armed[drone] = msg.armed

    # ----------------------------------------------------------------

    def position_callback(self, msg, drone):

        with self.lock:

            self.positions[drone] = (
                msg.pose.position.x,
                msg.pose.position.y,
                msg.pose.position.z
            )

    # ----------------------------------------------------------------

    def global_position_callback(self, msg, drone):

        # Ignore invalid GPS.

        if math.isnan(msg.latitude):
            return

        if math.isnan(msg.longitude):
            return

        with self.lock:

            self.global_positions[drone] = (
                msg.latitude,
                msg.longitude,
                msg.altitude
            )

    # ================================================================
    # CONNECTION CHECKS
    # ================================================================

    def all_connected(self):

        with self.lock:

            return all(
                self.connected[d]
                for d in self.drone_names
            )

    # ----------------------------------------------------------------

    def all_have_local_positions(self):

        with self.lock:

            return all(
                self.positions[d] is not None
                for d in self.drone_names
            )

    # ----------------------------------------------------------------

    def all_have_global_positions(self):

        with self.lock:

            return all(
                self.global_positions[d] is not None
                for d in self.drone_names
            )

    # ================================================================
    # SERVICE HELPERS
    # ================================================================

    def wait_future(self, future, timeout):

        start = time.time()

        while rclpy.ok():

            if future.done():

                try:
                    return future.result()
                except Exception as e:

                    self.get_logger().error(
                        f'Service exception: {e}'
                    )

                    return None

            if time.time() - start > timeout:

                return None

            time.sleep(0.05)

        return None

    # ----------------------------------------------------------------

    def arm_drone(self, drone):

        client = self.arm_clients[drone]

        if not client.wait_for_service(
            timeout_sec=3.0
        ):

            self.get_logger().error(
                f'[{drone}] arming service unavailable'
            )

            return False

        request = CommandBool.Request()

        request.value = True

        future = client.call_async(request)

        result = self.wait_future(
            future,
            5.0
        )

        if result is None:

            self.get_logger().error(
                f'[{drone}] ARM service failed'
            )

            return False

        if result.success:

            self.get_logger().info(
                f'[{drone}] ARM successful'
            )

            return True

        self.get_logger().error(
            f'[{drone}] ARM rejected'
        )

        return False

    # ================================================================

    def set_guided(self, drone):

        client = self.mode_clients[drone]

        if not client.wait_for_service(
            timeout_sec=3.0
        ):

            self.get_logger().error(
                f'[{drone}] set_mode service unavailable'
            )

            return False

        request = SetMode.Request()

        request.base_mode = 0

        request.custom_mode = 'GUIDED'

        future = client.call_async(request)

        result = self.wait_future(
            future,
            5.0
        )

        if result is None:

            self.get_logger().error(
                f'[{drone}] GUIDED request failed'
            )

            return False

        if result.mode_sent:

            self.get_logger().info(
                f'[{drone}] GUIDED requested'
            )

            return True

        self.get_logger().error(
            f'[{drone}] GUIDED rejected'
        )

        return False

    # ================================================================

    def takeoff_drone(self, drone):

        client = self.takeoff_clients[drone]

        if not client.wait_for_service(
            timeout_sec=3.0
        ):

            self.get_logger().error(
                f'[{drone}] takeoff service unavailable'
            )

            return False

        request = CommandTOL.Request()

        request.min_pitch = 0.0
        request.yaw = 0.0
        request.latitude = 0.0
        request.longitude = 0.0
        request.altitude = self.takeoff_altitude

        future = client.call_async(request)

        result = self.wait_future(
            future,
            10.0
        )

        if result is None:

            self.get_logger().error(
                f'[{drone}] takeoff service failed'
            )

            return False

        if result.success:

            self.get_logger().info(
                f'[{drone}] TAKEOFF accepted'
            )

            return True

        self.get_logger().error(
            f'[{drone}] TAKEOFF rejected'
        )

        return False

    # ================================================================

    def land_drone(self, drone):

        client = self.land_clients[drone]

        if not client.wait_for_service(
            timeout_sec=3.0
        ):

            self.get_logger().error(
                f'[{drone}] land service unavailable'
            )

            return False

        request = CommandTOL.Request()

        request.min_pitch = 0.0
        request.yaw = 0.0
        request.latitude = 0.0
        request.longitude = 0.0
        request.altitude = 0.0

        future = client.call_async(request)

        result = self.wait_future(
            future,
            10.0
        )

        if result is None:

            self.get_logger().error(
                f'[{drone}] LAND service failed'
            )

            return False

        if result.success:

            self.get_logger().info(
                f'[{drone}] LAND accepted'
            )

            return True

        self.get_logger().error(
            f'[{drone}] LAND rejected'
        )

        return False

    # ================================================================
    # GPS -> LOCAL COMMON COORDINATES
    # ================================================================

    def set_reference_position(self):

        if self.reference_lat is not None:
            return

        valid = [
            self.global_positions[d]
            for d in self.drone_names
            if self.global_positions[d] is not None
        ]

        if not valid:
            return

        self.reference_lat = sum(
            p[0] for p in valid
        ) / len(valid)

        self.reference_lon = sum(
            p[1] for p in valid
        ) / len(valid)

        self.get_logger().info(
            f'Common GPS reference: '
            f'lat={self.reference_lat:.8f}, '
            f'lon={self.reference_lon:.8f}'
        )

    # ----------------------------------------------------------------

    def gps_to_xy(self, lat, lon):

        """
        Convert GPS coordinates to local East/North meters.

        x = East
        y = North
        """

        if self.reference_lat is None:
            return None

        earth_radius = 6378137.0

        dlat = math.radians(
            lat - self.reference_lat
        )

        dlon = math.radians(
            lon - self.reference_lon
        )

        ref_lat_rad = math.radians(
            self.reference_lat
        )

        north = (
            dlat *
            earth_radius
        )

        east = (
            dlon *
            earth_radius *
            math.cos(ref_lat_rad)
        )

        return east, north

    # ----------------------------------------------------------------

    def get_common_position(self, drone):

        gps = self.global_positions[drone]

        if gps is None:
            return None

        return self.gps_to_xy(
            gps[0],
            gps[1]
        )

    # ================================================================
    # DISTANCE
    # ================================================================

    def distance_between(self, drone_a, drone_b):

        pa = self.get_common_position(drone_a)

        pb = self.get_common_position(drone_b)

        if pa is None or pb is None:

            return float('inf')

        dx = pa[0] - pb[0]

        dy = pa[1] - pb[1]

        return math.sqrt(
            dx * dx +
            dy * dy
        )

    # ================================================================
    # FORMATION DISTANCES
    # ================================================================

    def log_all_distances(self):

        for i in range(
            len(self.drone_names)
        ):

            for j in range(i + 1,
                           len(self.drone_names)):

                a = self.drone_names[i]

                b = self.drone_names[j]

                distance = self.distance_between(
                    a,
                    b
                )

                self.get_logger().info(
                    f'{a} <-> {b}: '
                    f'{distance:.2f} m'
                )

    # ================================================================
    # COLLISION AVOIDANCE
    # ================================================================

    def calculate_avoidance(self, drone):

        """
        GPS-based repulsive collision avoidance.

        The resulting vector is expressed in the same
        East/North frame for all three vehicles.

        Because the ArduPilot local frames in this SITL setup
        are aligned with the common world frame, East is treated
        as local X and North as local Y.
        """

        position = self.get_common_position(drone)

        if position is None:

            return 0.0, 0.0

        avoid_x = 0.0
        avoid_y = 0.0

        for other in self.drone_names:

            if other == drone:
                continue

            other_position = (
                self.get_common_position(other)
            )

            if other_position is None:
                continue

            dx = position[0] - other_position[0]

            dy = position[1] - other_position[1]

            distance = math.sqrt(
                dx * dx +
                dy * dy
            )

            # --------------------------------------------------------
            # No avoidance required.
            # --------------------------------------------------------

            if distance >= self.avoidance_distance:
                continue

            if distance < 0.001:

                continue

            # Direction away from other drone.

            ux = dx / distance

            uy = dy / distance

            # --------------------------------------------------------
            # Repulsive strength.
            #
            # At 7 m  -> approximately 0
            # At 5 m  -> strong
            # Below 5 -> emergency strength
            # --------------------------------------------------------

            if distance >= self.min_distance:

                strength = (
                    self.avoidance_distance - distance
                ) / (
                    self.avoidance_distance -
                    self.min_distance
                )

                strength = max(
                    0.0,
                    min(1.0, strength)
                )

            else:

                # Strong emergency response.

                emergency_ratio = (
                    self.min_distance - distance
                ) / self.min_distance

                strength = (
                    0.8 +
                    0.2 *
                    emergency_ratio
                )

                strength = min(
                    1.0,
                    strength
                )

                self.get_logger().error(
                    f'[{drone}] EMERGENCY AVOIDANCE '
                    f'from {other}: '
                    f'{distance:.2f} m'
                )

            avoid_x += (
                ux *
                self.max_avoidance_speed *
                strength
            )

            avoid_y += (
                uy *
                self.max_avoidance_speed *
                strength
            )

        # ------------------------------------------------------------
        # Limit avoidance velocity.
        # ------------------------------------------------------------

        magnitude = math.sqrt(
            avoid_x * avoid_x +
            avoid_y * avoid_y
        )

        if magnitude > self.max_avoidance_speed:

            scale = (
                self.max_avoidance_speed /
                magnitude
            )

            avoid_x *= scale
            avoid_y *= scale

        return avoid_x, avoid_y

    # ================================================================
    # EMERGENCY SEPARATION
    # ================================================================

    def emergency_separation_check(self):

        emergency = False

        for i in range(
            len(self.drone_names)
        ):

            for j in range(
                i + 1,
                len(self.drone_names)
            ):

                a = self.drone_names[i]

                b = self.drone_names[j]

                distance = self.distance_between(
                    a,
                    b
                )

                if distance < self.emergency_distance:

                    self.get_logger().error(
                        '!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!'
                    )

                    self.get_logger().error(
                        f'EMERGENCY SEPARATION: '
                        f'{a} <-> {b} = '
                        f'{distance:.2f} m'
                    )

                    self.get_logger().error(
                        'Applying maximum avoidance.'
                    )

                    self.get_logger().error(
                        '!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!'
                    )

                    emergency = True

        return emergency

    # ================================================================
    # VELOCITY COMMAND
    # ================================================================

    def publish_velocity(
        self,
        drone,
        vx,
        vy,
        vz=0.0
    ):

        # ------------------------------------------------------------
        # Limit total velocity.
        # ------------------------------------------------------------

        speed = math.sqrt(
            vx * vx +
            vy * vy +
            vz * vz
        )

        if speed > self.max_velocity:

            scale = (
                self.max_velocity /
                speed
            )

            vx *= scale
            vy *= scale
            vz *= scale

        msg = TwistStamped()

        msg.header.stamp = (
            self.get_clock()
            .now()
            .to_msg()
        )

        msg.twist.linear.x = vx

        msg.twist.linear.y = vy

        msg.twist.linear.z = vz

        msg.twist.angular.x = 0.0
        msg.twist.angular.y = 0.0
        msg.twist.angular.z = 0.0

        self.velocity_publishers[
            drone
        ].publish(msg)

    # ----------------------------------------------------------------

    def stop_drone(self, drone):

        self.publish_velocity(
            drone,
            0.0,
            0.0,
            0.0
        )

    # ----------------------------------------------------------------

    def stop_all(self):

        for drone in self.drone_names:

            self.stop_drone(drone)

    # ================================================================
    # WAIT FOR ALTITUDE
    # ================================================================

    def wait_for_altitude(self):

        self.get_logger().info(
            'Waiting for all drones to reach '
            f'{self.takeoff_altitude:.1f} m...'
        )

        start = time.time()

        while rclpy.ok():

            if time.time() - start > 40.0:

                self.get_logger().error(
                    'Altitude timeout.'
                )

                return False

            reached = True

            with self.lock:

                positions_copy = dict(
                    self.positions
                )

            for drone in self.drone_names:

                position = positions_copy[drone]

                if position is None:

                    reached = False
                    continue

                # MAVROS ENU:
                #
                # z normally becomes positive upward.
                #
                # We use absolute value to tolerate the
                # current SITL configuration.

                altitude = abs(
                    position[2]
                )

                if altitude < (
                    self.takeoff_altitude -
                    self.position_tolerance
                ):

                    reached = False

            if reached:

                self.get_logger().info(
                    'All drones reached takeoff altitude.'
                )

                return True

            time.sleep(0.1)

        return False

    # ================================================================
    # FORWARD FLIGHT
    # ================================================================

    def get_forward_distance(
        self,
        drone,
        start_position
    ):

        current = self.get_common_position(
            drone
        )

        if current is None:

            return 0.0

        dx = current[0] - start_position[0]

        dy = current[1] - start_position[1]

        return math.sqrt(
            dx * dx +
            dy * dy
        )

    # ----------------------------------------------------------------

    def formation_velocity(self, drone):

        # ------------------------------------------------------------
        # Normal forward velocity.
        #
        # Local X = forward.
        # ------------------------------------------------------------

        vx = self.forward_speed

        vy = 0.0

        # ------------------------------------------------------------
        # Collision avoidance.
        # ------------------------------------------------------------

        avoid_x, avoid_y = (
            self.calculate_avoidance(drone)
        )

        vx += avoid_x

        vy += avoid_y

        return vx, vy

    # ================================================================
    # MISSION
    # ================================================================

    def execute_mission(self):

        if self.mission_started:
            return

        self.mission_started = True

        # ============================================================
        # WAIT FOR FCUs
        # ============================================================

        self.get_logger().info(
            'Waiting for all three FCUs...'
        )

        start_wait = time.time()

        while rclpy.ok():

            if self.all_connected():

                break

            if time.time() - start_wait > 60.0:

                self.get_logger().error(
                    'FCU connection timeout.'
                )

                return

            time.sleep(0.1)

        self.get_logger().info(
            'All three FCUs connected.'
        )

        # ============================================================
        # WAIT FOR LOCAL POSITION
        # ============================================================

        self.get_logger().info(
            'Waiting for local positions...'
        )

        start_wait = time.time()

        while rclpy.ok():

            if self.all_have_local_positions():

                break

            if time.time() - start_wait > 30.0:

                self.get_logger().error(
                    'Local position timeout.'
                )

                return

            time.sleep(0.1)

        self.get_logger().info(
            'Local position available for all drones.'
        )

        # ============================================================
        # WAIT FOR GLOBAL POSITION
        # ============================================================

        self.get_logger().info(
            'Waiting for global GPS positions...'
        )

        start_wait = time.time()

        while rclpy.ok():

            if self.all_have_global_positions():

                break

            if time.time() - start_wait > 30.0:

                self.get_logger().error(
                    'GPS position timeout.'
                )

                return

            time.sleep(0.1)

        self.get_logger().info(
            'Global GPS position available for all drones.'
        )

        # ============================================================
        # COMMON REFERENCE
        # ============================================================

        self.set_reference_position()

        if self.reference_lat is None:

            self.get_logger().error(
                'Could not establish common GPS reference.'
            )

            return

        # ============================================================
        # INITIAL POSITIONS
        # ============================================================

        self.get_logger().info(
            'Initial formation:'
        )

        self.log_all_distances()

        # ============================================================
        # SAFETY CHECK
        # ============================================================

        if self.emergency_separation_check():

            self.get_logger().error(
                'Initial separation is too small.'
            )

            self.get_logger().error(
                'Mission ABORTED.'
            )

            return

        # ============================================================
        # SET GUIDED
        # ============================================================

        self.get_logger().info(
            'Switching all drones to GUIDED...'
        )

        for drone in self.drone_names:

            if not self.set_guided(drone):

                self.get_logger().error(
                    f'GUIDED failed for {drone}.'
                )

                self.land_all()

                return

            time.sleep(0.5)

        # ============================================================
        # ARM
        # ============================================================

        self.get_logger().info(
            'Arming all three drones...'
        )

        for drone in self.drone_names:

            if not self.arm_drone(drone):

                self.get_logger().error(
                    f'ARM failed for {drone}.'
                )

                self.land_all()

                return

            time.sleep(0.5)

        # ============================================================
        # TAKEOFF
        # ============================================================

        self.get_logger().info(
            'Taking off all three drones...'
        )

        for drone in self.drone_names:

            if not self.takeoff_drone(drone):

                self.get_logger().error(
                    f'TAKEOFF failed for {drone}.'
                )

                self.land_all()

                return

            time.sleep(0.5)

        # ============================================================
        # WAIT ALTITUDE
        # ============================================================

        if not self.wait_for_altitude():

            self.get_logger().error(
                'Takeoff altitude not reached.'
            )

            self.land_all()

            return

        # ============================================================
        # RECORD START POSITIONS
        # ============================================================

        start_positions = {}

        for drone in self.drone_names:

            start_positions[drone] = (
                self.get_common_position(drone)
            )

        self.get_logger().info(
            'Takeoff successful.'
        )

        self.log_all_distances()

        # ============================================================
        # FORWARD MISSION
        # ============================================================

        self.get_logger().info(
            '===================================================='
        )

        self.get_logger().info(
            '             FORMATION FLIGHT'
        )

        self.get_logger().info(
            '===================================================='
        )

        self.get_logger().info(
            f'Flying forward approximately '
            f'{self.forward_distance:.1f} m.'
        )

        self.get_logger().info(
            f'Normal speed: '
            f'{self.forward_speed:.1f} m/s.'
        )

        mission_start = time.time()

        last_distance_log = 0.0

        while rclpy.ok():

            if self.stop_requested:
                break

            # --------------------------------------------------------
            # Check progress
            # --------------------------------------------------------

            all_finished = True

            for drone in self.drone_names:

                start = start_positions[drone]

                if start is None:

                    all_finished = False

                    continue

                distance_forward = (
                    self.get_forward_distance(
                        drone,
                        start
                    )
                )

                if (
                    distance_forward <
                    self.forward_distance
                ):

                    all_finished = False

            if all_finished:

                break

            # --------------------------------------------------------
            # Collision avoidance
            # --------------------------------------------------------

            emergency = (
                self.emergency_separation_check()
            )

            # --------------------------------------------------------
            # Send commands
            # --------------------------------------------------------

            for drone in self.drone_names:

                vx, vy = (
                    self.formation_velocity(
                        drone
                    )
                )

                # If emergency condition exists,
                # normal forward motion is reduced.
                #
                # This gives the repulsive component priority.

                if emergency:

                    vx *= 0.25

                self.publish_velocity(
                    drone,
                    vx,
                    vy,
                    0.0
                )

            # --------------------------------------------------------
            # Periodic status
            # --------------------------------------------------------

            now = time.time()

            if now - last_distance_log > 2.0:

                self.log_all_distances()

                last_distance_log = now

            time.sleep(
                1.0 / self.control_rate
            )

        # ============================================================
        # STOP
        # ============================================================

        self.stop_all()

        self.get_logger().info(
            'Forward flight complete.'
        )

        # ============================================================
        # HOLD
        # ============================================================

        self.get_logger().info(
            f'Holding for '
            f'{self.hold_time:.1f} seconds...'
        )

        hold_start = time.time()

        while rclpy.ok():

            if (
                time.time() -
                hold_start
            ) >= self.hold_time:

                break

            # Keep sending zero velocity.

            for drone in self.drone_names:

                self.publish_velocity(
                    drone,
                    0.0,
                    0.0,
                    0.0
                )

            time.sleep(
                1.0 / self.control_rate
            )

        self.get_logger().info(
            '10-second hold complete.'
        )

        # ============================================================
        # LAND
        # ============================================================

        self.land_all()

        self.mission_finished = True

        self.get_logger().info(
            '===================================================='
        )

        self.get_logger().info(
            '              MISSION COMPLETE'
        )

        self.get_logger().info(
            '===================================================='
        )

    # ================================================================
    # LAND ALL
    # ================================================================

    def land_all(self):

        self.get_logger().warn(
            'Stopping velocity commands.'
        )

        self.stop_all()

        time.sleep(0.5)

        self.get_logger().warn(
            'Landing all drones...'
        )

        for drone in self.drone_names:

            self.land_drone(drone)

            time.sleep(0.5)

    # ================================================================
    # CONTROL TIMER
    # ================================================================

    def control_loop(self):

        if (
            not self.mission_started
            and not self.mission_finished
        ):

            threading.Thread(
                target=self.execute_mission,
                daemon=True
            ).start()


# ====================================================================
# MAIN
# ====================================================================

def main(args=None):

    rclpy.init(args=args)

    node = MultiDroneFormation()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        node.get_logger().warn(
            'CTRL+C received.'
        )

        node.stop_requested = True

        node.stop_all()

        time.sleep(0.5)

        node.land_all()

    finally:

        node.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':

    main()
