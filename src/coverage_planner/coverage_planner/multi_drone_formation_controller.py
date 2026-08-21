#!/usr/bin/env python3

import math
import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL, StreamRate
from sensor_msgs.msg import NavSatFix


class MultiDroneFormation(Node):

    def __init__(self):
        super().__init__('multi_drone_formation_controller')

        self.cb_group = ReentrantCallbackGroup()

        # Drones setup
        self.drone_names = ['drone1', 'drone2', 'drone3']

        # Flight parameters
        self.takeoff_altitude = 5.0
        self.forward_speed = 1.0
        self.forward_distance = 15.0
        self.hold_time = 10.0

        # Collision parameters (Meters)
        self.min_distance = 4.0
        self.avoidance_distance = 6.0
        self.emergency_distance = 3.0
        self.max_avoidance_speed = 1.5
        self.max_velocity = 2.0

        # Control tolerances
        self.position_tolerance = 0.5
        self.control_rate = 20.0
        self.connection_timeout = 60.0
        self.position_timeout = 45.0

        # State storage
        self.states = {}
        self.positions = {}
        self.global_positions = {}
        self.connected = {}
        self.ever_connected = {}
        self.armed = {}

        self.velocity_publishers = {}
        self.arm_clients = {}
        self.mode_clients = {}
        self.takeoff_clients = {}
        self.land_clients = {}
        self.stream_clients = {}

        self.lock = threading.Lock()
        self.stop_requested = False

        # QoS configuration tailored for ArduPilot / MAVROS
        mavros_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        for drone in self.drone_names:
            self.states[drone] = State()
            self.positions[drone] = None
            self.global_positions[drone] = None
            self.connected[drone] = False
            self.ever_connected[drone] = False
            self.armed[drone] = False

            # STATE
            self.create_subscription(
                State,
                f'/{drone}/state',
                self.make_state_cb(drone),
                mavros_qos,
                callback_group=self.cb_group
            )

            # LOCAL POS
            self.create_subscription(
                PoseStamped,
                f'/{drone}/local_position/pose',
                self.make_pos_cb(drone),
                mavros_qos,
                callback_group=self.cb_group
            )

            # GLOBAL POS
            self.create_subscription(
                NavSatFix,
                f'/{drone}/global_position/global',
                self.make_global_cb(drone),
                mavros_qos,
                callback_group=self.cb_group
            )

            # PUBLISHER
            self.velocity_publishers[drone] = self.create_publisher(
                TwistStamped,
                f'/{drone}/setpoint_velocity/cmd_vel',
                10
            )

            # CLIENTS
            self.arm_clients[drone] = self.create_client(
                CommandBool, f'/{drone}/cmd/arming', callback_group=self.cb_group
            )
            self.mode_clients[drone] = self.create_client(
                SetMode, f'/{drone}/set_mode', callback_group=self.cb_group
            )
            self.takeoff_clients[drone] = self.create_client(
                CommandTOL, f'/{drone}/cmd/takeoff', callback_group=self.cb_group
            )
            self.land_clients[drone] = self.create_client(
                CommandTOL, f'/{drone}/cmd/land', callback_group=self.cb_group
            )
            self.stream_clients[drone] = self.create_client(
                StreamRate, f'/{drone}/set_stream_rate', callback_group=self.cb_group
            )

        self.get_logger().info('MULTI-DRONE FORMATION CONTROLLER INITIALIZED')

    # Callback Generators
    def make_state_cb(self, drone):
        def cb(msg):
            with self.lock:
                self.states[drone] = msg
                self.connected[drone] = msg.connected
                if msg.connected:
                    self.ever_connected[drone] = True
                self.armed[drone] = msg.armed
        return cb

    def make_pos_cb(self, drone):
        def cb(msg):
            with self.lock:
                self.positions[drone] = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
        return cb

    def make_global_cb(self, drone):
        def cb(msg):
            with self.lock:
                self.global_positions[drone] = (msg.latitude, msg.longitude, msg.altitude)
        return cb

    # Math/Distance Helpers
    @staticmethod
    def haversine_distance(lat1, lon1, lat2, lon2):
        R = 6371000.0
        phi1, phi2 = math.radians(lat1), math.radians(lat2)
        dphi = math.radians(lat2 - lat1)
        dlam = math.radians(lon2 - lon1)
        a = math.sin(dphi / 2)**2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlam / 2)**2
        return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

    def get_gps_offset_meters(self, drone_a, drone_b):
        ga, gb = self.global_positions[drone_a], self.global_positions[drone_b]
        if ga is None or gb is None:
            return 0.0, 0.0
        dy = self.haversine_distance(ga[0], ga[1], gb[0], ga[1]) * (1.0 if gb[0] > ga[0] else -1.0)
        dx = self.haversine_distance(ga[0], ga[1], ga[0], gb[1]) * (1.0 if gb[1] > ga[1] else -1.0)
        return dx, dy

    def distance_between(self, drone_a, drone_b):
        ga, gb = self.global_positions[drone_a], self.global_positions[drone_b]
        if ga is None or gb is None:
            return float('inf')
        h_dist = self.haversine_distance(ga[0], ga[1], gb[0], gb[1])
        dz = ga[2] - gb[2]
        return math.sqrt(h_dist**2 + dz**2)

    # Service Caller
    def call_service_sync(self, client, request, timeout=3.0):
        if not client.service_is_ready():
            if not client.wait_for_service(timeout_sec=2.0):
                return None

        event = threading.Event()
        response_container = []

        def done_callback(future):
            try:
                response_container.append(future.result())
            except Exception as e:
                self.get_logger().error(f'Service call failed: {e}')
                response_container.append(None)
            event.set()

        future = client.call_async(request)
        future.add_done_callback(done_callback)

        if event.wait(timeout=timeout):
            return response_container[0]
        return None

    def enable_ardu_streams(self, drone):
        req = StreamRate.Request()
        req.stream_id = 0
        req.message_rate = 10
        req.on_off = True
        self.call_service_sync(self.stream_clients[drone], req, timeout=2.0)

    def set_guided(self, drone):
        req = SetMode.Request()
        req.base_mode = 0

        # Numerical mode '4' works best natively in ArduPilot MAVROS
        for mode in ['4', 'GUIDED']:
            req.custom_mode = mode
            res = self.call_service_sync(self.mode_clients[drone], req, timeout=2.0)
            if res and res.mode_sent:
                self.get_logger().info(f'[{drone}] Set mode to {mode} successfully.')
                return True
        return False

    def arm_drone(self, drone):
        req = CommandBool.Request()
        req.value = True
        res = self.call_service_sync(self.arm_clients[drone], req, timeout=5.0)
        return res is not None and res.success

    def takeoff_drone(self, drone):
        req = CommandTOL.Request()
        req.altitude = float(self.takeoff_altitude)
        res = self.call_service_sync(self.takeoff_clients[drone], req, timeout=10.0)
        return res is not None and res.success

    def land_drone(self, drone):
        req = CommandTOL.Request()
        res = self.call_service_sync(self.land_clients[drone], req, timeout=10.0)
        return res is not None and res.success

    def publish_velocity(self, drone, vx, vy, vz=0.0):
        speed = math.sqrt(vx**2 + vy**2 + vz**2)
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

    def stop_all(self):
        for drone in self.drone_names:
            self.publish_velocity(drone, 0.0, 0.0, 0.0)

    def land_all(self):
        self.stop_all()
        time.sleep(0.5)
        self.get_logger().warn('Landing all drones...')
        for drone in self.drone_names:
            self.land_drone(drone)
            time.sleep(0.5)

    def calculate_avoidance(self, drone):
        avoid_x, avoid_y = 0.0, 0.0
        for other in self.drone_names:
            if other == drone:
                continue

            dist = self.distance_between(drone, other)
            if dist >= self.avoidance_distance or dist < 0.001:
                continue

            dx, dy = self.get_gps_offset_meters(drone, other)
            ux, uy = dx / dist, dy / dist

            if dist >= self.min_distance:
                ratio = (self.avoidance_distance - dist) / (self.avoidance_distance - self.min_distance)
                strength = max(0.0, min(1.0, ratio))
            else:
                emergency_ratio = (self.min_distance - dist) / self.min_distance
                strength = 0.75 + 0.25 * emergency_ratio

            avoid_x += ux * self.max_avoidance_speed * strength
            avoid_y += uy * self.max_avoidance_speed * strength

        mag = math.sqrt(avoid_x**2 + avoid_y**2)
        if mag > self.max_avoidance_speed:
            avoid_x = (avoid_x / mag) * self.max_avoidance_speed
            avoid_y = (avoid_y / mag) * self.max_avoidance_speed

        return avoid_x, avoid_y

    def formation_velocity(self, drone):
        forward_x, forward_y = self.forward_speed, 0.0
        avoid_x, avoid_y = self.calculate_avoidance(drone)

        nearest_dist = min([self.distance_between(drone, o) for o in self.drone_names if o != drone])

        if nearest_dist < self.min_distance:
            return avoid_x, avoid_y

        if nearest_dist < self.avoidance_distance:
            ratio = max(0.0, min(1.0, (nearest_dist - self.min_distance) / (self.avoidance_distance - self.min_distance)))
            forward_x *= ratio

        return forward_x + avoid_x, forward_y + avoid_y

    # Mission Process
    def execute_mission(self):
        self.get_logger().info('Waiting for FCU connections...')
        start = time.time()
        last_log = time.time()

        while rclpy.ok() and (time.time() - start < self.connection_timeout):
            with self.lock:
                if all(self.connected[d] or self.ever_connected[d] for d in self.drone_names):
                    self.get_logger().info('All FCUs confirmed connected!')
                    break

            if time.time() - last_log > 5.0:
                status_str = ", ".join([f"{d}: {'OK' if (self.connected[d] or self.ever_connected[d]) else 'WAIT'}" for d in self.drone_names])
                self.get_logger().info(f'Connection status -> {status_str}')
                last_log = time.time()

            time.sleep(0.5)
        else:
            self.get_logger().error('FCU Connection Timeout!')
            return

        # Request streams after verifying initial connection
        for d in self.drone_names:
            self.enable_ardu_streams(d)

        self.get_logger().info('Waiting for GPS fixes...')
        start = time.time()
        while rclpy.ok() and (time.time() - start < self.position_timeout):
            with self.lock:
                if all(v is not None for v in self.global_positions.values()) and all(p is not None for p in self.positions.values()):
                    self.get_logger().info('GPS Fix & Local Pose acquired for all drones.')
                    break
            time.sleep(0.5)

        for i in range(len(self.drone_names)):
            for j in range(i + 1, len(self.drone_names)):
                a, b = self.drone_names[i], self.drone_names[j]
                self.get_logger().info(f'Distance {a} <-> {b}: {self.distance_between(a, b):.2f} m')

        self.get_logger().info('Priming setpoint streams (30 stream bursts)...')
        for _ in range(30):
            self.stop_all()
            time.sleep(0.05)

        self.get_logger().info('Setting GUIDED mode for all drones...')
        for d in self.drone_names:
            if not self.set_guided(d):
                self.get_logger().warn(f'Failed to set GUIDED for {d} pre-arm. Retrying after arming sequence...')

        self.get_logger().info('Arming all drones...')
        for d in self.drone_names:
            if not self.arm_drone(d):
                self.get_logger().error(f'Failed to ARM {d}')
                return
            time.sleep(0.3)

        # Retry setting GUIDED if drone required arming first
        self.get_logger().info('Verifying GUIDED mode post-arm...')
        for d in self.drone_names:
            if self.states[d].mode not in ['GUIDED', 'CMODE(4)']:
                if not self.set_guided(d):
                    self.get_logger().error(f'Failed to set GUIDED mode post-arm for {d}')
                    return

        self.get_logger().info('Initiating Takeoff...')
        for d in self.drone_names:
            if not self.takeoff_drone(d):
                self.get_logger().error(f'Takeoff failed for {d}')
                self.land_all()
                return
            time.sleep(0.3)

        self.get_logger().info('Reaching takeoff altitude...')
        start = time.time()
        while rclpy.ok() and (time.time() - start < 45.0):
            if self.stop_requested:
                return
            reached = all([abs(self.positions[d][2]) >= (self.takeoff_altitude - self.position_tolerance) for d in self.drone_names])
            if reached:
                self.get_logger().info('All drones reached target altitude!')
                break
            time.sleep(0.2)

        self.get_logger().info('Starting Formation Forward Movement...')
        start_positions = {d: self.positions[d] for d in self.drone_names}
        last_log = time.time()

        while rclpy.ok() and not self.stop_requested:
            all_finished = True
            for d in self.drone_names:
                curr = self.positions[d]
                start_p = start_positions[d]
                fwd_dist = math.sqrt((curr[0] - start_p[0])**2 + (curr[1] - start_p[1])**2)
                if fwd_dist < self.forward_distance:
                    all_finished = False

            if all_finished:
                break

            for d in self.drone_names:
                vx, vy = self.formation_velocity(d)
                self.publish_velocity(d, vx, vy, 0.0)

            if time.time() - last_log > 2.0:
                d12 = self.distance_between('drone1', 'drone2')
                d23 = self.distance_between('drone2', 'drone3')
                self.get_logger().info(f'Separation: D1-D2 = {d12:.2f}m | D2-D3 = {d23:.2f}m')
                last_log = time.time()

            time.sleep(1.0 / self.control_rate)

        self.get_logger().info(f'Forward target reached. Holding position for {self.hold_time} seconds...')
        hold_start = time.time()
        while rclpy.ok() and (time.time() - hold_start < self.hold_time) and not self.stop_requested:
            self.stop_all()
            time.sleep(1.0 / self.control_rate)

        self.get_logger().info('Mission Complete! Landing...')
        self.land_all()

def main(args=None):
    rclpy.init(args=args)
    node = MultiDroneFormation()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    thread = threading.Thread(target=node.execute_mission, daemon=True)
    thread.start()

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.stop_requested = True
        node.land_all()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
