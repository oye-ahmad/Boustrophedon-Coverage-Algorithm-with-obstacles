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
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from sensor_msgs.msg import NavSatFix


class MultiDroneFormation(Node):

    def __init__(self):
        super().__init__('multi_drone_formation_controller')

        self.cb_group = ReentrantCallbackGroup()

        self.drone_names = ['drone1', 'drone2', 'drone3']

        self.takeoff_altitude = 5.0
        self.forward_speed = 1.0
        self.forward_distance = 15.0
        self.hold_time = 10.0

        self.min_distance = 5.0
        self.avoidance_distance = 7.0
        self.emergency_distance = 4.5
        self.max_avoidance_speed = 1.5
        self.max_velocity = 2.0

        self.position_tolerance = 0.5
        self.control_rate = 20.0
        self.connection_timeout = 60.0
        self.position_timeout = 30.0

        self.states = {}
        self.positions = {}
        self.global_positions = {}

        self.connected = {}
        self.armed = {}

        self.velocity_publishers = {}
        self.arm_clients = {}
        self.mode_clients = {}
        self.takeoff_clients = {}
        self.land_clients = {}

        self.lock = threading.Lock()
        self.stop_requested = False

        # QoS configuration
        state_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

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
            self.armed[drone] = False

            # STATE
            self.create_subscription(
                State,
                f'/{drone}/state',
                self.make_state_cb(drone),
                state_qos,
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

        self.get_logger().info('MULTI-DRONE FORMATION CONTROLLER INITIALIZED')

    # Explicit Callback Handlers
    def make_state_cb(self, drone):
        def cb(msg):
            with self.lock:
                self.states[drone] = msg
                self.connected[drone] = msg.connected
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

    # Distance Functions
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

    # Mission Logic
    def execute_mission(self):
        self.get_logger().info('Waiting for FCU connections...')
        start = time.time()
        while rclpy.ok() and (time.time() - start < self.connection_timeout):
            with self.lock:
                if all(self.connected.values()):
                    self.get_logger().info('All FCUs connected!')
                    break
            time.sleep(0.2)
        else:
            self.get_logger().error('FCU Connection Timeout!')
            return

        self.get_logger().info('Waiting for GPS fixes...')
        start = time.time()
        while rclpy.ok() and (time.time() - start < self.position_timeout):
            with self.lock:
                if all(v is not None for v in self.global_positions.values()):
                    self.get_logger().info('GPS Fix acquired for all drones.')
                    break
            time.sleep(0.2)

        for i in range(len(self.drone_names)):
            for j in range(i + 1, len(self.drone_names)):
                a, b = self.drone_names[i], self.drone_names[j]
                self.get_logger().info(f'Distance {a} <-> {b}: {self.distance_between(a, b):.2f} m')

        self.get_logger().info('Mission initialized. Ready for commands.')

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
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
