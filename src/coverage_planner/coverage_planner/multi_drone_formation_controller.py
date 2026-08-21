#!/usr/bin/env python3

import math
import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy,
    DurabilityPolicy
)

from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from sensor_msgs.msg import NavSatFix


class MultiDroneFormation(Node):

    def __init__(self):
        super().__init__('multi_drone_formation_controller')

        # Multi-threaded callback group to prevent worker deadlocks
        self.cb_group = ReentrantCallbackGroup()

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
        # COLLISION PARAMETERS
        # ------------------------------------------------------------

        self.min_distance = 5.0
        self.avoidance_distance = 7.0
        self.emergency_distance = 4.5
        self.max_avoidance_speed = 1.5
        self.max_velocity = 2.0

        # ------------------------------------------------------------
        # CONTROL
        # ------------------------------------------------------------

        self.position_tolerance = 0.5
        self.control_rate = 20.0
        self.connection_timeout = 60.0
        self.position_timeout = 30.0

        # ============================================================
        # STATE
        # ============================================================

        self.states = {}
        self.positions = {}
        self.home_positions = {}
        self.global_positions = {}

        self.connected = {}
        self.armed = {}

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
        # QoS
        # ============================================================

        mavros_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        state_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # ============================================================
        # CREATE ROS INTERFACES
        # ============================================================

        for drone in self.drone_names:

            self.states[drone] = State()
            self.positions[drone] = None
            self.global_positions[drone] = None
            self.home_positions[drone] = None
            self.connected[drone] = False
            self.armed[drone] = False

            # STATE
            self.create_subscription(
                State,
                f'/{drone}/state',
                lambda msg, d=drone: self.state_callback(msg, d),
                state_qos,
                callback_group=self.cb_group
            )

            # LOCAL POSITION
            self.create_subscription(
                PoseStamped,
                f'/{drone}/local_position/pose',
                lambda msg, d=drone: self.position_callback(msg, d),
                mavros_qos,
                callback_group=self.cb_group
            )

            # GLOBAL POSITION
            self.create_subscription(
                NavSatFix,
                f'/{drone}/global_position/global',
                lambda msg, d=drone: self.global_position_callback(msg, d),
                mavros_qos,
                callback_group=self.cb_group
            )

            # VELOCITY PUBLISHER
            self.velocity_publishers[drone] = self.create_publisher(
                TwistStamped,
                f'/{drone}/setpoint_velocity/cmd_vel',
                10
            )

            # CLIENTS
            self.arm_clients[drone] = self.create_client(
                CommandBool,
                f'/{drone}/cmd/arming',
                callback_group=self.cb_group
            )

            self.mode_clients[drone] = self.create_client(
                SetMode,
                f'/{drone}/set_mode',
                callback_group=self.cb_group
            )

            self.takeoff_clients[drone] = self.create_client(
                CommandTOL,
                f'/{drone}/cmd/takeoff',
                callback_group=self.cb_group
            )

            self.land_clients[drone] = self.create_client(
                CommandTOL,
                f'/{drone}/cmd/land',
                callback_group=self.cb_group
            )

        # ============================================================
        # LOG CONFIGURATION
        # ============================================================

        self.get_logger().info('====================================================')
        self.get_logger().info('     MULTI-DRONE FORMATION CONTROLLER')
        self.get_logger().info('====================================================')
        self.get_logger().info('Drones: drone1, drone2, drone3')
        self.get_logger().info(f'Takeoff altitude: {self.takeoff_altitude:.1f} m')
        self.get_logger().info(f'Forward speed: {self.forward_speed:.1f} m/s')
        self.get_logger().info(f'Forward distance: {self.forward_distance:.1f} m')
        self.get_logger().info(f'Minimum separation: {self.min_distance:.1f} m')
        self.get_logger().info(f'Avoidance starts: {self.avoidance_distance:.1f} m')
        self.get_logger().info(f'Emergency distance: {self.emergency_distance:.1f} m')
        self.get_logger().info(f'Hold time: {self.hold_time:.1f} seconds')
        self.get_logger().info('Using local ENU coordinates for collision avoidance.')
        self.get_logger().info('Waiting for all drones...')

        # Start execution thread directly
        threading.Thread(target=self.execute_mission, daemon=True).start()

    # ================================================================
    # CALLBACKS
    # ================================================================

    def state_callback(self, msg, drone):
        with self.lock:
            self.states[drone] = msg
            self.connected[drone] = msg.connected
            self.armed[drone] = msg.armed

    def position_callback(self, msg, drone):
        with self.lock:
            self.positions[drone] = (
                msg.pose.position.x,
                msg.pose.position.y,
                msg.pose.position.z
            )

    def global_position_callback(self, msg, drone):
        with self.lock:
            self.global_positions[drone] = (
                msg.latitude,
                msg.longitude,
                msg.altitude
            )

    # ================================================================
    # CONNECTION HELPERS
    # ================================================================

    def all_connected(self):
        with self.lock:
            return all(self.connected[d] for d in self.drone_names)

    def all_have_positions(self):
        with self.lock:
            return all(self.positions[d] is not None for d in self.drone_names)

    def all_have_global_positions(self):
        with self.lock:
            return all(self.global_positions[d] is not None for d in self.drone_names)

    # ================================================================
    # WAIT HELPERS
    # ================================================================

    def wait_for_connections(self):
        self.get_logger().info('Waiting for all three FCUs...')
        start = time.time()

        while rclpy.ok():
            if self.stop_requested:
                return False

            if self.all_connected():
                self.get_logger().info('All three FCUs connected.')
                return True

            elapsed = time.time() - start
            if elapsed > self.connection_timeout:
                self.get_logger().error('FCU connection timeout.')
                for drone in self.drone_names:
                    if not self.connected[drone]:
                        self.get_logger().error(f'[{drone}] FCU NOT connected')
                return False

            time.sleep(0.1)
        return False

    def wait_for_positions(self):
        self.get_logger().info('Waiting for local positions...')
        start = time.time()

        while rclpy.ok():
            if self.stop_requested:
                return False

            if self.all_have_positions():
                self.get_logger().info('Local position available for all drones.')
                return True

            if time.time() - start > self.position_timeout:
                self.get_logger().error('Local position timeout.')
                for drone in self.drone_names:
                    if self.positions[drone] is None:
                        self.get_logger().error(f'[{drone}] local position unavailable')
                return False

            time.sleep(0.1)
        return False

    def wait_for_global_positions(self):
        self.get_logger().info('Waiting for global GPS positions...')
        start = time.time()

        while rclpy.ok():
            if self.stop_requested:
                return False

            if self.all_have_global_positions():
                self.get_logger().info('Global GPS position available for all drones.')
                return True

            if time.time() - start > self.position_timeout:
                self.get_logger().warn('Global GPS position timeout.')
                return False

            time.sleep(0.1)
        return False

    def wait_for_services(self):
        self.get_logger().info('Checking MAVROS services...')
        service_map = {
            'arming': self.arm_clients,
            'set_mode': self.mode_clients,
            'takeoff': self.takeoff_clients,
            'land': self.land_clients
        }

        for service_name, clients in service_map.items():
            for drone in self.drone_names:
                client = clients[drone]
                if not client.wait_for_service(timeout_sec=10.0):
                    self.get_logger().error(f'[{drone}] {service_name} service unavailable')
                    return False
                self.get_logger().info(f'[{drone}] {service_name} service available')

        self.get_logger().info('All required MAVROS services are available.')
        return True

    # ================================================================
    # SERVICE ACTIONS
    # ================================================================

    def arm_drone(self, drone):
        client = self.arm_clients[drone]
        request = CommandBool.Request()
        request.value = True

        future = client.call_async(request)
        start = time.time()

        while rclpy.ok():
            if future.done():
                break
            if time.time() - start > 5.0:
                self.get_logger().error(f'[{drone}] ARM timeout')
                return False
            time.sleep(0.05)

        result = future.result()
        if result and result.success:
            self.get_logger().info(f'[{drone}] ARM successful')
            return True

        self.get_logger().error(f'[{drone}] ARM rejected')
        return False

    def set_guided(self, drone):
        client = self.mode_clients[drone]
        request = SetMode.Request()
        request.base_mode = 0
        request.custom_mode = 'GUIDED'

        self.get_logger().info(f'[{drone}] Requesting GUIDED...')
        future = client.call_async(request)
        start = time.time()

        while rclpy.ok():
            if future.done():
                break
            if time.time() - start > 5.0:
                self.get_logger().error(f'[{drone}] GUIDED request timeout')
                return False
            time.sleep(0.05)

        result = future.result()
        if result and result.mode_sent:
            self.get_logger().info(f'[{drone}] GUIDED command accepted')
            return True

        self.get_logger().error(f'[{drone}] GUIDED rejected')
        return False

    def takeoff_drone(self, drone):
        client = self.takeoff_clients[drone]
        request = CommandTOL.Request()
        request.min_pitch = 0.0
        request.yaw = 0.0
        request.latitude = 0.0
        request.longitude = 0.0
        request.altitude = self.takeoff_altitude

        self.get_logger().info(f'[{drone}] Requesting takeoff...')
        future = client.call_async(request)
        start = time.time()

        while rclpy.ok():
            if future.done():
                break
            if time.time() - start > 10.0:
                self.get_logger().error(f'[{drone}] TAKEOFF timeout')
                return False
            time.sleep(0.05)

        result = future.result()
        if result and result.success:
            self.get_logger().info(f'[{drone}] TAKEOFF accepted')
            return True

        self.get_logger().error(f'[{drone}] TAKEOFF rejected')
        return False

    def land_drone(self, drone):
        client = self.land_clients[drone]
        request = CommandTOL.Request()
        request.min_pitch = 0.0
        request.yaw = 0.0
        request.latitude = 0.0
        request.longitude = 0.0
        request.altitude = 0.0

        self.get_logger().info(f'[{drone}] Requesting LAND...')
        future = client.call_async(request)
        start = time.time()

        while rclpy.ok():
            if future.done():
                break
            if time.time() - start > 10.0:
                self.get_logger().error(f'[{drone}] LAND timeout')
                return False
            time.sleep(0.05)

        result = future.result()
        if result and result.success:
            self.get_logger().info(f'[{drone}] LAND accepted')
            return True

        self.get_logger().error(f'[{drone}] LAND rejected')
        return False

    # ================================================================
    # VELOCITY & DISTANCE
    # ================================================================

    def publish_velocity(self, drone, vx, vy, vz=0.0):
        speed = math.sqrt(vx * vx + vy * vy + vz * vz)
        if speed > self.max_velocity:
            scale = self.max_velocity / speed
            vx *= scale
            vy *= scale
            vz *= scale

        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x = float(vx)
        msg.twist.linear.y = float(vy)
        msg.twist.linear.z = float(vz)

        self.velocity_publishers[drone].publish(msg)

    def stop_drone(self, drone):
        self.publish_velocity(drone, 0.0, 0.0, 0.0)

    def stop_all(self):
        for drone in self.drone_names:
            self.stop_drone(drone)

    def distance_between(self, drone_a, drone_b):
        pa = self.positions[drone_a]
        pb = self.positions[drone_b]

        if pa is None or pb is None:
            return float('inf')

        dx = pa[0] - pb[0]
        dy = pa[1] - pb[1]
        dz = pa[2] - pb[2]

        return math.sqrt(dx * dx + dy * dy + dz * dz)

    def print_initial_formation(self):
        self.get_logger().info('Initial formation:')
        for i in range(len(self.drone_names)):
            for j in range(i + 1, len(self.drone_names)):
                a = self.drone_names[i]
                b = self.drone_names[j]
                distance = self.distance_between(a, b)
                self.get_logger().info(f'{a} <-> {b}: {distance:.2f} m')

    def calculate_avoidance(self, drone):
        position = self.positions[drone]
        if position is None:
            return 0.0, 0.0

        avoid_x = 0.0
        avoid_y = 0.0

        for other in self.drone_names:
            if other == drone:
                continue

            other_position = self.positions[other]
            if other_position is None:
                continue

            dx = position[0] - other_position[0]
            dy = position[1] - other_position[1]
            distance = math.sqrt(dx * dx + dy * dy)

            if distance >= self.avoidance_distance or distance < 0.001:
                continue

            ux = dx / distance
            uy = dy / distance

            if distance >= self.min_distance:
                ratio = (self.avoidance_distance - distance) / (self.avoidance_distance - self.min_distance)
                strength = max(0.0, min(1.0, ratio))
            else:
                emergency_ratio = (self.min_distance - distance) / self.min_distance
                strength = 0.75 + 0.25 * emergency_ratio
                self.get_logger().warn(f'[{drone}] CLOSE TO {other}: {distance:.2f} m')

            avoid_x += ux * self.max_avoidance_speed * strength
            avoid_y += uy * self.max_avoidance_speed * strength

        magnitude = math.sqrt(avoid_x * avoid_x + avoid_y * avoid_y)
        if magnitude > self.max_avoidance_speed:
            scale = self.max_avoidance_speed / magnitude
            avoid_x *= scale
            avoid_y *= scale

        return avoid_x, avoid_y

    def formation_velocity(self, drone):
        forward_x = self.forward_speed
        forward_y = 0.0
        avoid_x, avoid_y = self.calculate_avoidance(drone)

        nearest_distance = float('inf')
        for other in self.drone_names:
            if other == drone:
                continue
            d = self.distance_between(drone, other)
            nearest_distance = min(nearest_distance, d)

        if nearest_distance < self.emergency_distance:
            self.get_logger().error(f'[{drone}] EMERGENCY SEPARATION: {nearest_distance:.2f} m')
            return avoid_x, avoid_y

        if nearest_distance < self.min_distance:
            return avoid_x, avoid_y

        if nearest_distance < self.avoidance_distance:
            ratio = (nearest_distance - self.min_distance) / (self.avoidance_distance - self.min_distance)
            ratio = max(0.0, min(1.0, ratio))
            forward_x *= ratio

        return forward_x + avoid_x, forward_y + avoid_y

    def emergency_separation_check(self):
        emergency = False
        for i in range(len(self.drone_names)):
            for j in range(i + 1, len(self.drone_names)):
                a = self.drone_names[i]
                b = self.drone_names[j]
                distance = self.distance_between(a, b)
                if distance < self.emergency_distance:
                    self.get_logger().error(f'!!! EMERGENCY SEPARATION !!! {a} <-> {b}: {distance:.2f} m')
                    emergency = True
        return emergency

    def wait_for_altitude(self):
        self.get_logger().info('Waiting for all drones to reach takeoff altitude...')
        start = time.time()

        while rclpy.ok():
            if self.stop_requested:
                return False

            reached = True
            for drone in self.drone_names:
                position = self.positions[drone]
                if position is None:
                    reached = False
                    continue

                altitude = abs(position[2])
                if altitude < (self.takeoff_altitude - self.position_tolerance):
                    reached = False

            if reached:
                self.get_logger().info('All drones reached takeoff altitude.')
                return True

            if time.time() - start > 45.0:
                self.get_logger().error('Takeoff altitude timeout.')
                return False

            time.sleep(0.1)
        return False

    def land_all(self):
        self.get_logger().warn('Stopping velocity commands.')
        self.stop_all()
        time.sleep(0.5)

        self.get_logger().warn('Landing all drones...')
        for drone in self.drone_names:
            self.land_drone(drone)
            time.sleep(0.5)

    # ================================================================
    # MISSION EXECUTION
    # ================================================================

    def execute_mission(self):
        if not self.wait_for_connections():
            return

        if not self.wait_for_positions():
            return

        self.wait_for_global_positions()

        with self.lock:
            for drone in self.drone_names:
                self.home_positions[drone] = self.positions[drone]

        self.print_initial_formation()

        if self.emergency_separation_check():
            self.get_logger().error('Initial formation is unsafe.')
            return

        if not self.wait_for_services():
            self.get_logger().error('Required MAVROS services are unavailable.')
            return

        self.get_logger().info('Switching all drones to GUIDED...')
        for drone in self.drone_names:
            if not self.set_guided(drone):
                self.get_logger().error(f'GUIDED failed for {drone}.')
                self.stop_all()
                self.land_all()
                return
            time.sleep(0.5)

        self.get_logger().info('Arming all drones...')
        for drone in self.drone_names:
            if not self.arm_drone(drone):
                self.get_logger().error(f'ARM failed for {drone}.')
                self.stop_all()
                self.land_all()
                return
            time.sleep(0.5)

        self.get_logger().info('Taking off all three drones...')
        for drone in self.drone_names:
            if not self.takeoff_drone(drone):
                self.get_logger().error(f'TAKEOFF failed for {drone}.')
                self.land_all()
                return
            time.sleep(0.5)

        if not self.wait_for_altitude():
            self.get_logger().error('Takeoff altitude was not reached.')
            self.land_all()
            return

        self.get_logger().info('====================================================')
        self.get_logger().info('          FORMATION FLIGHT STARTED')
        self.get_logger().info('====================================================')

        start_positions = {}
        with self.lock:
            for drone in self.drone_names:
                start_positions[drone] = self.positions[drone]

        last_log = time.time()

        while rclpy.ok():
            if self.stop_requested:
                break

            all_finished = True
            for drone in self.drone_names:
                start = start_positions[drone]
                current = self.positions[drone]

                if current is None:
                    all_finished = False
                    continue

                dx = current[0] - start[0]
                dy = current[1] - start[1]
                forward_distance = math.sqrt(dx * dx + dy * dy)

                if forward_distance < self.forward_distance:
                    all_finished = False

            if all_finished:
                break

            self.emergency_separation_check()

            for drone in self.drone_names:
                vx, vy = self.formation_velocity(drone)
                self.publish_velocity(drone, vx, vy, 0.0)

            if time.time() - last_log > 2.0:
                d12 = self.distance_between('drone1', 'drone2')
                d13 = self.distance_between('drone1', 'drone3')
                d23 = self.distance_between('drone2', 'drone3')
                self.get_logger().info(f'Separation: D1-D2={d12:.2f} m | D1-D3={d13:.2f} m | D2-D3={d23:.2f} m')
                last_log = time.time()

            time.sleep(1.0 / self.control_rate)

        self.stop_all()
        self.get_logger().info('====================================================')
        self.get_logger().info('Forward mission complete.')
        self.get_logger().info(f'Holding for {self.hold_time:.1f} seconds.')
        self.get_logger().info('====================================================')

        hold_start = time.time()
        while rclpy.ok():
            if self.stop_requested:
                break
            if time.time() - hold_start >= self.hold_time:
                break

            for drone in self.drone_names:
                self.publish_velocity(drone, 0.0, 0.0, 0.0)

            time.sleep(1.0 / self.control_rate)

        self.get_logger().info('10-second hold complete.')
        self.land_all()
        self.mission_finished = True
        self.get_logger().info('====================================================')
        self.get_logger().info('              MISSION COMPLETE')
        self.get_logger().info('====================================================')


# ====================================================================
# MAIN
# ====================================================================

def main(args=None):
    rclpy.init(args=args)
    node = MultiDroneFormation()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().warn('CTRL+C received.')
        node.stop_requested = True
        node.stop_all()
        time.sleep(0.5)
        node.land_all()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
