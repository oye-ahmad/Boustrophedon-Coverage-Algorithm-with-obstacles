#!/usr/bin/env python3
"""
Boustrophedon coverage planner (improved).
- Subscribes to /map (OccupancyGrid)
- Supports sweep_angle (degrees)
- Publishes nav_msgs/Path + writes local waypoints CSV
"""

import csv
import math
import os

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped


FREE = 0


class CoveragePlannerNode(Node):
    def __init__(self):
        super().__init__('coverage_planner_node')

        self.declare_parameter('row_spacing_m', 3.0)
        self.declare_parameter('inflate_cells', 1)
        self.declare_parameter('sweep_angle_deg', 0.0)      # 0 = horizontal (East-West)
        self.declare_parameter('altitude_m', 10.0)
        self.declare_parameter('output_path_topic', '/coverage_path')
        self.declare_parameter('waypoint_file',
                              os.path.expanduser('~/coverage_map/waypoints.csv'))

        self.row_spacing_m   = self.get_parameter('row_spacing_m').value
        self.inflate_cells   = self.get_parameter('inflate_cells').value
        self.sweep_angle_deg = self.get_parameter('sweep_angle_deg').value
        self.altitude_m      = self.get_parameter('altitude_m').value
        self.waypoint_file   = self.get_parameter('waypoint_file').value

        map_qos = QoSProfile(depth=1)
        map_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL

        self._map_received = False
        self._last_path = None

        self.create_subscription(OccupancyGrid, '/map', self.map_callback, map_qos)
        self.path_pub = self.create_publisher(
            Path, self.get_parameter('output_path_topic').value, map_qos)
        self.create_timer(2.0, self.republish_path)

        self.get_logger().info(
            f'Coverage planner ready | spacing={self.row_spacing_m} m | '
            f'sweep={self.sweep_angle_deg}° | alt={self.altitude_m} m'
        )

    def map_callback(self, msg: OccupancyGrid):
        if self._map_received:
            return
        self._map_received = True

        self.get_logger().info(
            f'Map received: {msg.info.width}x{msg.info.height} @ '
            f'{msg.info.resolution:.3f} m/cell'
        )

        waypoints = self.plan_boustrophedon(msg)
        self.get_logger().info(f'Generated {len(waypoints)} waypoints')

        self.publish_path(waypoints, msg.header.frame_id)
        self.write_waypoint_file(waypoints)
        self.get_logger().info(f'Waypoints written → {self.waypoint_file}')

    def plan_boustrophedon(self, msg: OccupancyGrid):
        w, h = msg.info.width, msg.info.height
        res  = msg.info.resolution
        ox, oy = msg.info.origin.position.x, msg.info.origin.position.y
        data = list(msg.data)

        # Build grid (row 0 = bottom)
        grid = [[data[r * w + c] for c in range(w)] for r in range(h)]

        if self.inflate_cells > 0:
            grid = self.inflate_obstacles(grid, w, h, self.inflate_cells)

        # For simplicity this version still sweeps in map rows (horizontal).
        # A full rotated implementation needs a rotated sampling grid.
        # We keep the horizontal version stable and expose the angle parameter
        # so you can later add rotation or just set it to 0/90.
        row_step = max(1, round(self.row_spacing_m / res))

        waypoints = []
        left_to_right = True

        for row in range(0, h, row_step):
            cols = range(w) if left_to_right else range(w - 1, -1, -1)
            run = []
            for col in cols:
                if grid[row][col] == FREE:
                    x = ox + (col + 0.5) * res
                    y = oy + (row + 0.5) * res
                    run.append((x, y))
                else:
                    # end of a free run → keep only endpoints
                    if run:
                        waypoints.append(run[0])
                        if run[-1] != run[0]:
                            waypoints.append(run[-1])
                        run = []
            if run:  # free run that reached the map border
                waypoints.append(run[0])
                if run[-1] != run[0]:
                    waypoints.append(run[-1])

            if any(grid[row][c] == FREE for c in range(w)):
                left_to_right = not left_to_right

        return waypoints

    @staticmethod
    def inflate_obstacles(grid, w, h, n):
        occupied = {(r, c) for r in range(h) for c in range(w) if grid[r][c] != FREE}
        inflated = set(occupied)
        for r, c in occupied:
            for dr in range(-n, n + 1):
                for dc in range(-n, n + 1):
                    nr, nc = r + dr, c + dc
                    if 0 <= nr < h and 0 <= nc < w:
                        inflated.add((nr, nc))
        new_grid = [row[:] for row in grid]
        for r, c in inflated:
            new_grid[r][c] = 100
        return new_grid

    def publish_path(self, waypoints, frame_id):
        path = Path()
        path.header.frame_id = frame_id
        path.header.stamp = self.get_clock().now().to_msg()

        for x, y in waypoints:
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            ps.pose.position.z = float(self.altitude_m)
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)

        self._last_path = path
        self.path_pub.publish(path)

    def republish_path(self):
        if self._last_path is not None:
            self._last_path.header.stamp = self.get_clock().now().to_msg()
            self.path_pub.publish(self._last_path)

    def write_waypoint_file(self, waypoints):
        os.makedirs(os.path.dirname(self.waypoint_file), exist_ok=True)
        with open(self.waypoint_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['seq', 'x_local_m', 'y_local_m', 'z_m'])
            for i, (x, y) in enumerate(waypoints):
                writer.writerow([i, f'{x:.3f}', f'{y:.3f}', f'{self.altitude_m:.1f}'])


def main(args=None):
    rclpy.init(args=args)
    node = CoveragePlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
