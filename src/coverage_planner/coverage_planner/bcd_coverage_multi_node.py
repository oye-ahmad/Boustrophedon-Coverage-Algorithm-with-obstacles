#!/usr/bin/env python3
"""
bcd_coverage_multi_node.py

Self-contained 3-drone Boustrophedon Cellular Decomposition (BCD) coverage
planner. Everything - map handling, obstacle inflation, column-wise cell
decomposition, lawnmower generation, greedy sequencing, cell visualization,
AND the 3-way region partitioning / stitching / CSV output - lives in this
single file. No imports from other node files.

Subscribes:
  /map   (nav_msgs/OccupancyGrid)

Publishes:
  /bcd_cells          (visualization_msgs/MarkerArray)  - cell outlines/ids
  /coverage_path_1     (nav_msgs/Path)
  /coverage_path_2     (nav_msgs/Path)
  /coverage_path_3     (nav_msgs/Path)

Writes:
  waypoints_drone1.csv, waypoints_drone2.csv, waypoints_drone3.csv
  (columns: seq, x_local_m, y_local_m, z_m)
"""

import os
import csv
import math
from dataclasses import dataclass, field
from typing import List, Dict, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile

from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA


FREE = 0
OCCUPIED = 100
N_DRONES = 3


@dataclass
class Cell:
    id: int
    # list of (col, row_start, row_end) inclusive, in grid coordinates
    slices: List[Tuple[int, int, int]] = field(default_factory=list)
    min_col: int = 999999
    max_col: int = -1
    center: Tuple[float, float] = (0.0, 0.0)

    def add_slice(self, col: int, r1: int, r2: int):
        self.slices.append((col, r1, r2))
        self.min_col = min(self.min_col, col)
        self.max_col = max(self.max_col, col)

    def compute_center(self, res: float, ox: float, oy: float):
        if not self.slices:
            return
        xs, ys = [], []
        for col, r1, r2 in self.slices:
            x = ox + (col + 0.5) * res
            for r in range(r1, r2 + 1):
                y = oy + (r + 0.5) * res
                xs.append(x)
                ys.append(y)
        self.center = (sum(xs) / len(xs), sum(ys) / len(ys))


# ============================================================
# Standalone region-partitioning helpers (formerly region_partitioner.py)
# ============================================================
def _path_length(path: List[Tuple[float, float]]) -> float:
    total = 0.0
    for (x1, y1), (x2, y2) in zip(path, path[1:]):
        total += math.hypot(x2 - x1, y2 - y1)
    return total


def partition_cells(
    ordered_ids: List[int],
    cell_paths: Dict[int, List[Tuple[float, float]]],
    n_drones: int = 3,
) -> List[List[int]]:
    """
    Split ordered_ids into n_drones contiguous groups, balanced by total
    path length. Returns a list of n_drones cell-id lists.
    """
    if n_drones < 1:
        raise ValueError("n_drones must be >= 1")
    if not ordered_ids:
        return [[] for _ in range(n_drones)]

    lengths = {cid: _path_length(cell_paths[cid]) for cid in ordered_ids}
    total_length = sum(lengths.values())
    target_per_region = total_length / n_drones if total_length > 0 else 0

    regions: List[List[int]] = []
    current: List[int] = []
    running_total = 0.0
    region_index = 0

    for cid in ordered_ids:
        current.append(cid)
        running_total += lengths[cid]
        if (
            region_index < n_drones - 1
            and running_total >= target_per_region * (region_index + 1)
        ):
            regions.append(current)
            current = []
            region_index += 1

    if current or len(regions) < n_drones:
        regions.append(current)
    while len(regions) < n_drones:
        regions.append([])

    return regions


def stitch_region_path(
    region_cell_ids: List[int],
    cell_paths: Dict[int, List[Tuple[float, float]]],
) -> List[Tuple[float, float]]:
    """
    Concatenate a region's cell paths in order, inserting a straight-line
    transition between the end of one cell and the start of the next.
    """
    full_path: List[Tuple[float, float]] = []
    prev_end = None
    for cid in region_cell_ids:
        path = cell_paths[cid]
        if not path:
            continue
        if prev_end is not None:
            full_path.append(prev_end)
            full_path.append(path[0])
        full_path.extend(path)
        prev_end = path[-1]
    return full_path


def region_summary(
    regions: List[List[int]],
    cell_paths: Dict[int, List[Tuple[float, float]]],
    logger=None,
) -> None:
    for i, region in enumerate(regions, start=1):
        total_len = sum(_path_length(cell_paths[cid]) for cid in region)
        msg = f"Drone {i}: {len(region)} cells {region}, total path length ~{total_len:.1f} m"
        if logger is not None:
            logger.info(msg)
        else:
            print(msg)


# ============================================================
# Main node
# ============================================================
class BCDCoverageMultiNode(Node):
    def __init__(self):
        super().__init__('bcd_coverage_multi_node')

        # ---- Parameters ----
        self.declare_parameter('row_spacing_m', 3.0)
        self.declare_parameter('inflate_cells', 1)
        self.declare_parameter('altitude_m', 10.0)
        self.declare_parameter('waypoint_file',
                                os.path.expanduser('~/coverage_map/waypoints.csv'))
        self.declare_parameter('cell_marker_topic', '/bcd_cells')

        self.row_spacing_m = self.get_parameter('row_spacing_m').value
        self.inflate_cells = self.get_parameter('inflate_cells').value
        self.altitude_m = self.get_parameter('altitude_m').value
        self.waypoint_file = self.get_parameter('waypoint_file').value
        self.waypoint_dir = os.path.dirname(self.waypoint_file)

        # ---- QoS ----
        map_qos = QoSProfile(
            depth=10,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )

        self._map_received = False
        self._cells: List[Cell] = []
        self._last_region_paths: Dict[int, Path] = {}

        # ---- Subscriptions / Publishers ----
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, map_qos)

        self.cell_pub = self.create_publisher(
            MarkerArray, self.get_parameter('cell_marker_topic').value, map_qos)

        self.region_path_pubs = {
            i: self.create_publisher(Path, f'/coverage_path_{i}', map_qos)
            for i in range(1, N_DRONES + 1)
        }

        self.create_timer(2.0, self.republish)

        self.get_logger().info(
            f'BCD Multi-Drone Coverage Planner started ({N_DRONES} drones). Waiting for /map ...')

    # ------------------------------------------------------------------
    def map_callback(self, msg: OccupancyGrid):
        if self._map_received:
            return
        self._map_received = True

        self.get_logger().info(
            f'Map: {msg.info.width}x{msg.info.height} @ {msg.info.resolution:.3f} m')

        grid = self.msg_to_grid(msg)
        if self.inflate_cells > 0:
            grid = self.inflate_obstacles(grid, self.inflate_cells)

        # 1. Decompose
        self._cells = self.boustrophedon_decomposition(grid)
        self.get_logger().info(f'Decomposed into {len(self._cells)} cells')

        # 2. Compute centers
        res = msg.info.resolution
        ox = msg.info.origin.position.x
        oy = msg.info.origin.position.y
        for cell in self._cells:
            cell.compute_center(res, ox, oy)

        # 3. Visualize cells
        self.publish_cells(msg)

        # 4. Per-cell lawnmower generation
        row_step = max(1, int(round(self.row_spacing_m / res)))
        cell_paths: Dict[int, List[Tuple[float, float]]] = {}
        for cell in self._cells:
            path = self.lawnmower_in_cell(cell, grid, res, ox, oy, row_step)
            if path:
                cell_paths[cell.id] = path

        if not cell_paths:
            self.get_logger().warn('No coverable cells found.')
            return

        # 5. Greedy nearest-neighbor sequencing
        ordered_ids = self.greedy_sequence(cell_paths)

        # 6. Partition sequence into N_DRONES contiguous, balanced regions
        regions = partition_cells(ordered_ids, cell_paths, n_drones=N_DRONES)
        region_summary(regions, cell_paths, logger=self.get_logger())

        # 7. Stitch, publish, write CSV per drone
        for i, region_cell_ids in enumerate(regions, start=1):
            region_path = stitch_region_path(region_cell_ids, cell_paths)
            self.get_logger().info(f'Drone {i}: {len(region_path)} waypoints')
            self.publish_region_path(i, region_path, msg.header.frame_id)
            self.write_region_csv(i, region_path)

        self.get_logger().info('3-drone coverage paths published and written to CSV.')

    # ------------------------------------------------------------------
    def msg_to_grid(self, msg: OccupancyGrid):
        w, h = msg.info.width, msg.info.height
        data = list(msg.data)
        return [[data[r * w + c] for c in range(w)] for r in range(h)]

    @staticmethod
    def inflate_obstacles(grid, n):
        h = len(grid)
        w = len(grid[0])
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
            new_grid[r][c] = OCCUPIED
        return new_grid

    # ======================== BCD CORE ========================
    def boustrophedon_decomposition(self, grid) -> List[Cell]:
        """
        Classic grid-based Boustrophedon Cellular Decomposition.
        Sweeps column by column and creates monotone cells.
        """
        h = len(grid)
        w = len(grid[0]) if h > 0 else 0

        cells: List[Cell] = []
        next_id = 0

        prev_segments = []

        for col in range(w):
            curr_segments = []
            r = 0
            while r < h:
                if grid[r][col] == FREE:
                    r1 = r
                    while r < h and grid[r][col] == FREE:
                        r += 1
                    r2 = r - 1
                    curr_segments.append([r1, r2, None])
                else:
                    r += 1

            used_prev = [False] * len(prev_segments)

            for seg in curr_segments:
                r1, r2, _ = seg
                best_idx = -1
                best_overlap = 0

                for i, (pr1, pr2, cid) in enumerate(prev_segments):
                    if used_prev[i]:
                        continue
                    overlap = max(0, min(r2, pr2) - max(r1, pr1) + 1)
                    if overlap > best_overlap:
                        best_overlap = overlap
                        best_idx = i

                if best_idx >= 0 and best_overlap > 0:
                    cid = prev_segments[best_idx][2]
                    seg[2] = cid
                    used_prev[best_idx] = True
                    cells[cid].add_slice(col, r1, r2)
                else:
                    seg[2] = next_id
                    new_cell = Cell(id=next_id)
                    new_cell.add_slice(col, r1, r2)
                    cells.append(new_cell)
                    next_id += 1

            prev_segments = [(s[0], s[1], s[2]) for s in curr_segments]

        cells = [c for c in cells if c.slices]
        return cells

    # ======================== PATH PLANNING ========================
    def lawnmower_in_cell(self, cell: Cell, grid, res, ox, oy, row_step):
        """Generate alternating horizontal lawnmower path inside one cell."""
        free_points = set()
        for col, r1, r2 in cell.slices:
            for r in range(r1, r2 + 1):
                free_points.add((col, r))

        if not free_points:
            return []

        rows = sorted(set(r for c, r in free_points))
        selected_rows = rows[::row_step]
        if not selected_rows:
            selected_rows = rows

        waypoints = []
        left_to_right = True

        for r in selected_rows:
            cols = sorted(c for c, rr in free_points if rr == r)
            if not cols:
                continue
            if left_to_right:
                c1, c2 = cols[0], cols[-1]
            else:
                c1, c2 = cols[-1], cols[0]

            x1 = ox + (c1 + 0.5) * res
            x2 = ox + (c2 + 0.5) * res
            y = oy + (r + 0.5) * res

            waypoints.append((x1, y))
            if abs(x2 - x1) > 1e-3:
                waypoints.append((x2, y))

            left_to_right = not left_to_right

        return waypoints

    def greedy_sequence(self, cell_paths: Dict[int, List]):
        """Nearest-neighbor ordering of cells."""
        if not cell_paths:
            return []

        ids = list(cell_paths.keys())
        ordered = [ids[0]]
        remaining = set(ids[1:])

        while remaining:
            last_id = ordered[-1]
            last_end = cell_paths[last_id][-1]
            best = None
            best_dist = float('inf')
            for cid in remaining:
                start = cell_paths[cid][0]
                d = math.hypot(start[0] - last_end[0], start[1] - last_end[1])
                if d < best_dist:
                    best_dist = d
                    best = cid
            ordered.append(best)
            remaining.remove(best)

        return ordered

    # ======================== VISUALIZATION ========================
    def publish_cells(self, msg: OccupancyGrid):
        res = msg.info.resolution
        ox = msg.info.origin.position.x
        oy = msg.info.origin.position.y
        frame = msg.header.frame_id

        ma = MarkerArray()

        delete = Marker()
        delete.action = Marker.DELETEALL
        ma.markers.append(delete)

        colors = self._generate_colors(len(self._cells))

        for idx, cell in enumerate(self._cells):
            m = Marker()
            m.header.frame_id = frame
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'bcd_cells'
            m.id = cell.id
            m.type = Marker.LINE_STRIP
            m.action = Marker.ADD
            m.scale.x = 0.15
            m.color = colors[idx]
            m.pose.orientation.w = 1.0

            points = []
            for col, r1, r2 in cell.slices:
                x = ox + (col + 0.5) * res
                y1 = oy + (r1 + 0.5) * res
                y2 = oy + (r2 + 0.5) * res
                points.append((x, y1))
                points.append((x, y2))

            for x, y in points:
                p = Point()
                p.x = x
                p.y = y
                p.z = 0.1
                m.points.append(p)

            ma.markers.append(m)

            t = Marker()
            t.header = m.header
            t.ns = 'bcd_cell_ids'
            t.id = 1000 + cell.id
            t.type = Marker.TEXT_VIEW_FACING
            t.action = Marker.ADD
            t.scale.z = 0.8
            t.color.r = t.color.g = t.color.b = t.color.a = 1.0
            t.pose.position.x = cell.center[0]
            t.pose.position.y = cell.center[1]
            t.pose.position.z = 0.5
            t.pose.orientation.w = 1.0
            t.text = str(cell.id)
            ma.markers.append(t)

        self.cell_pub.publish(ma)
        self.get_logger().info(f'Published {len(self._cells)} cell markers')

    def _generate_colors(self, n):
        colors = []
        for i in range(n):
            hue = i / max(1, n)
            r = abs(hue * 6 - 3) - 1
            g = 2 - abs(hue * 6 - 2)
            b = 2 - abs(hue * 6 - 4)
            c = ColorRGBA()
            c.r = max(0.0, min(1.0, r))
            c.g = max(0.0, min(1.0, g))
            c.b = max(0.0, min(1.0, b))
            c.a = 0.9
            colors.append(c)
        return colors

    # ======================== OUTPUT ========================
    def publish_region_path(self, drone_id: int, waypoints, frame_id):
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

        self._last_region_paths[drone_id] = path
        self.region_path_pubs[drone_id].publish(path)

    def write_region_csv(self, drone_id: int, waypoints):
        filename = os.path.join(self.waypoint_dir, f'waypoints_drone{drone_id}.csv')
        os.makedirs(self.waypoint_dir, exist_ok=True)
        with open(filename, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['seq', 'x_local_m', 'y_local_m', 'z_m'])
            for i, (x, y) in enumerate(waypoints):
                writer.writerow([i, f'{x:.3f}', f'{y:.3f}', f'{self.altitude_m:.1f}'])
        self.get_logger().info(f'Wrote {len(waypoints)} waypoints -> {filename}')

    def republish(self):
        for drone_id, path in self._last_region_paths.items():
            path.header.stamp = self.get_clock().now().to_msg()
            self.region_path_pubs[drone_id].publish(path)


def main(args=None):
    rclpy.init(args=args)
    node = BCDCoverageMultiNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
