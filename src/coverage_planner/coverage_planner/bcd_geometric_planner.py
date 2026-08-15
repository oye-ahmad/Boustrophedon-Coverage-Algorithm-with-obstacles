#!/usr/bin/env python3
import os
import csv
import numpy as np
import matplotlib.pyplot as plt
from typing import List, Tuple, Dict, Optional

# ROS2 Libraries
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA

# Computational Geometry & Geodesy
import shapely.geometry as sg
import shapely.ops as so
from shapely.errors import TopologicalError
import pyproj

# ==============================================================================
# Helper Classes & Geodesy Utilities
# ==============================================================================

# Minimum vertical extent for a slice/interval to be considered real (filters
# out degenerate slivers caused by near-duplicate x-coordinates).
MIN_SLICE_HEIGHT = 0.05


class Cell:
    """Represents an x-monotone cell constructed during sweep-line decomposition."""
    def __init__(self, cell_id: int, start_x: float):
        self.cell_id = cell_id
        self.start_x = start_x
        self.end_x: Optional[float] = None
        # List of vertical slices: (x, y_bottom, y_top)
        self.slices: List[Tuple[float, float, float]] = []
        self.polygon: Optional[sg.Polygon] = None

    def add_slice(self, x: float, y_bottom: float, y_top: float):
        self.slices.append((x, y_bottom, y_top))

    def close(self, end_x: float):
        self.end_x = end_x
        if not self.slices:
            return

        # Sort slices by x coordinate (should already be in order, but be safe)
        self.slices.sort(key=lambda s: s[0])

        # Build boundary vertices (bottom edge left->right, top edge right->left)
        bottom_pts = [(s[0], s[1]) for s in self.slices]
        top_pts = [(s[0], s[2]) for s in reversed(self.slices)]

        points = bottom_pts + top_pts

        if len(points) < 4:
            self.polygon = None
            return

        # Remove consecutive duplicate points
        clean_points = []
        for p in points:
            if not clean_points or p != clean_points[-1]:
                clean_points.append(p)

        # Polygon needs at least 3 unique points
        if len(set(clean_points)) < 3:
            self.polygon = None
            return

        try:
            raw_poly = sg.Polygon(clean_points)

            if not raw_poly.is_valid:
                # Only fall back to buffer(0) as a last resort - if the
                # matching logic upstream is correct, this should rarely fire.
                raw_poly = raw_poly.buffer(0)

            if raw_poly.is_empty or raw_poly.area < 1e-6:
                self.polygon = None
            else:
                self.polygon = raw_poly

        except Exception:
            self.polygon = None


class ENUToGeodeticConverter:
    """Converts local ENU Cartesian coordinates (meters) to global Geodetic coordinates (Lat/Lon)."""
    def __init__(self, lat0: float, lon0: float):
        self.lat0 = lat0
        self.lon0 = lon0
        # Set up a local Transverse Mercator projection centered at the map origin
        proj_string = f"+proj=tmerc +lat_0={lat0} +lon_0={lon0} +k=1 +x_0=0 +y_0=0 +ellps=WGS84 +units=m +no_defs"
        self.transformer_to_geo = pyproj.Transformer.from_proj(proj_string, "EPSG:4326", always_xy=True)

    def enu_to_geodetic(self, x: float, y: float, z: float) -> Tuple[float, float, float]:
        lon, lat = self.transformer_to_geo.transform(x, y)
        return lat, lon, z


# ==============================================================================
# BCD Geometric Planner Node
# ==============================================================================

class BCDGeometricPlanner(Node):
    def __init__(self):
        super().__init__('bcd_geometric_planner')

        # Configuration Parameters
        self.map_origin_lat = 37.7749
        self.map_origin_lon = -122.4194
        self.fly_altitude = 10.0
        self.row_spacing = 3.0
        self.eps = 1e-4

        self.converter = ENUToGeodeticConverter(self.map_origin_lat, self.map_origin_lon)

        # Environment Definitions (100m x 100m boundary)
        self.boundary = sg.Polygon([(0, 0), (100, 0), (100, 100), (0, 100)])

        # Simplified to 2 plain rectangular obstacles. Each rectangle still
        # produces one SPLIT event (entering) and one MERGE event (exiting)
        # as the sweep line crosses it, so the algorithm is fully exercised
        # without the extra complexity of L/U-shaped obstacles.
        self.obstacle_a = sg.Polygon([(15, 30), (25, 30), (25, 70), (15, 70)])
        self.obstacle_b = sg.Polygon([(60, 20), (75, 20), (75, 80), (60, 80)])

        self.obstacles = [self.obstacle_a, self.obstacle_b]

        # ROS2 Publishers
        self.path_pub = self.create_publisher(Path, '/coverage_path', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/bcd_cells', 10)

        self.get_logger().info("BCD Planner Node initialized. Starting decomposition...")

    def execute_plan(self):
        """Main execution workflow."""
        try:
            cells = self._compute_bcd_cells()
            self.get_logger().info(f"Decomposition complete. Total x-monotone cells created: {len(cells)}")

            full_path_pts = self._generate_coverage_path(cells)
            self.get_logger().info(f"Coverage path generated. Total waypoints: {len(full_path_pts)}")

            # Export Artifacts
            self._export_to_csv(full_path_pts)
            self._publish_ros_data(cells, full_path_pts)
            self._generate_debug_plot(cells, full_path_pts)

        except Exception as e:
            self.get_logger().error(f"Critical error during planning execution: {str(e)}")

    # --------------------------------------------------------------------------
    # Core BCD Algorithm Implementation
    # --------------------------------------------------------------------------

    def _compute_bcd_cells(self) -> List[Cell]:
        """Implements Choset's exact sweep-line BCD algorithm."""
        # Step 1: Collect unique critical x-coordinates
        x_coords = set()
        for pt in self.boundary.exterior.coords:
            x_coords.add(round(pt[0], 6))
        for obs in self.obstacles:
            for pt in obs.exterior.coords:
                x_coords.add(round(pt[0], 6))

        sorted_x = sorted(list(x_coords))

        # Collapse near-duplicate critical x-values before sweeping, so we
        # never generate a near-zero-width column (source of sliver cells).
        dedup_x = [sorted_x[0]]
        for x in sorted_x[1:]:
            if x - dedup_x[-1] > 1e-3:
                dedup_x.append(x)
        sorted_x = dedup_x

        closed_cells: List[Cell] = []
        active_cells: List[Cell] = []
        cell_counter = 0

        # Sweep-line iterations across critical slices
        for i in range(len(sorted_x) - 1):
            x_curr = sorted_x[i]
            x_next = sorted_x[i + 1]

            # Evaluate slice mid-point to avoid exact vertex intersection ambiguities
            x_mid = (x_curr + x_next) / 2.0

            # Compute free-space segments at x_mid
            free_intervals = self._get_free_intervals_at(x_mid)

            if not active_cells:
                # Initial creation (IN Event). Add slices at BOTH ends of the
                # column (not just the midpoint) so that even a cell whose
                # entire lifetime is this single column ends up with 2
                # slices (4 points) - the minimum needed for a valid polygon.
                for interval in free_intervals:
                    cell = Cell(cell_counter, x_curr)
                    cell_counter += 1
                    cell.add_slice(x_curr, interval[0], interval[1])
                    cell.add_slice(x_next, interval[0], interval[1])
                    active_cells.append(cell)
            else:
                matched_pairs, split_events, merge_events = self._match_intervals(active_cells, free_intervals)

                # Handle Merges (cells that did not survive into the new column).
                # Their slice list already reaches x_curr (added as the
                # "x_next" of the previous column), so no new slice is needed
                # here - just finalize the polygon.
                for cell in merge_events:
                    cell.close(x_curr)
                    closed_cells.append(cell)
                    active_cells.remove(cell)

                # Update Continuing Cells - add a point at both the start and
                # end of this column. Adjacent columns sharing the same
                # interval will produce a duplicate point at the shared
                # boundary x, which Cell.close() collapses harmlessly.
                for cell, interval in matched_pairs:
                    cell.add_slice(x_curr, interval[0], interval[1])
                    cell.add_slice(x_next, interval[0], interval[1])

                # Handle Splits (new intervals with no surviving parent cell).
                # Same reasoning as the IN-event case: seed both endpoints so
                # a split-then-immediately-merged cell (common with a narrow
                # obstacle that spans exactly one column) is still valid.
                for interval in split_events:
                    new_cell = Cell(cell_counter, x_curr)
                    cell_counter += 1
                    new_cell.add_slice(x_curr, interval[0], interval[1])
                    new_cell.add_slice(x_next, interval[0], interval[1])
                    active_cells.append(new_cell)

        # Close any remaining active cells at boundary right edge
        for cell in active_cells:
            cell.close(sorted_x[-1])
            closed_cells.append(cell)

        return [c for c in closed_cells if c.polygon is not None and not c.polygon.is_empty]

    def _get_free_intervals_at(self, x: float) -> List[Tuple[float, float]]:
        """Intersects a vertical line slice at x with obstacles and boundary to obtain free intervals."""
        vertical_line = sg.LineString([(x, -10.0), (x, 110.0)])

        try:
            boundary_cut = self.boundary.intersection(vertical_line)
            if boundary_cut.is_empty:
                return []

            y_min_b, y_max_b = boundary_cut.bounds[1], boundary_cut.bounds[3]
            occupied_segments = []

            for obs in self.obstacles:
                cut = obs.intersection(vertical_line)
                if cut.is_empty:
                    continue
                if isinstance(cut, sg.LineString):
                    occupied_segments.append((cut.bounds[1], cut.bounds[3]))
                elif isinstance(cut, sg.MultiLineString):
                    for geom in cut.geoms:
                        occupied_segments.append((geom.bounds[1], geom.bounds[3]))

            # Merge overlapping occupied segments
            occupied_segments.sort(key=lambda seg: seg[0])
            merged_occ = []
            for seg in occupied_segments:
                if not merged_occ:
                    merged_occ.append(list(seg))
                else:
                    if seg[0] <= merged_occ[-1][1] + 1e-6:
                        merged_occ[-1][1] = max(merged_occ[-1][1], seg[1])
                    else:
                        merged_occ.append(list(seg))

            # Compute inverse complement within boundary limits
            free_intervals = []
            curr_y = y_min_b
            for occ in merged_occ:
                if occ[0] > curr_y + 1e-4:
                    free_intervals.append((curr_y, occ[0]))
                curr_y = max(curr_y, occ[1])

            if y_max_b > curr_y + 1e-4:
                free_intervals.append((curr_y, y_max_b))

            # Drop degenerate slivers
            free_intervals = [iv for iv in free_intervals if (iv[1] - iv[0]) > MIN_SLICE_HEIGHT]

            return free_intervals

        except TopologicalError:
            return []

    def _match_intervals(self, active_cells: List[Cell], intervals: List[Tuple[float, float]]):
        """
        Matches active cells to new intervals to identify CONTINUATION, SPLIT,
        and MERGE events.

        Unlike a naive independent-max-overlap match (which can assign two
        intervals to the same cell, or swap assignments between adjacent
        cells and produce self-intersecting polygons), this sorts both cells
        and intervals by their vertical (y) position and matches greedily in
        that rank order. This preserves top-to-bottom ordering across the
        sweep and prevents crossed/duplicate assignments.
        """
        matched: List[Tuple[Cell, Tuple[float, float]]] = []
        splits: List[Tuple[float, float]] = []
        merges: List[Cell] = []

        if not active_cells:
            return matched, list(intervals), merges

        if not intervals:
            return matched, splits, list(active_cells)

        def cell_mid(c: Cell) -> float:
            last = c.slices[-1]
            return (last[1] + last[2]) / 2.0

        def interval_overlap(c: Cell, iv: Tuple[float, float]) -> float:
            last = c.slices[-1]
            return max(0.0, min(iv[1], last[2]) - max(iv[0], last[1]))

        sorted_cells = sorted(active_cells, key=cell_mid)
        sorted_intervals = sorted(intervals, key=lambda iv: (iv[0] + iv[1]) / 2.0)

        n_cells = len(sorted_cells)
        n_intervals = len(sorted_intervals)

        if n_cells == n_intervals:
            # Simple continuation: pair by rank, but only if there is real
            # overlap (guards against a pathological same-count coincidence
            # that isn't actually a continuation).
            for cell, interval in zip(sorted_cells, sorted_intervals):
                if interval_overlap(cell, interval) > 1e-3:
                    matched.append((cell, interval))
                else:
                    merges.append(cell)
                    splits.append(interval)

        elif n_intervals > n_cells:
            # One or more SPLIT events: each existing cell claims its best
            # remaining interval (searched in rank order so ties resolve
            # consistently); anything left over is a brand-new cell.
            used_intervals = set()
            for cell in sorted_cells:
                best_idx, best_overlap = None, 1e-3
                for idx, interval in enumerate(sorted_intervals):
                    if idx in used_intervals:
                        continue
                    ov = interval_overlap(cell, interval)
                    if ov > best_overlap:
                        best_overlap = ov
                        best_idx = idx
                if best_idx is not None:
                    matched.append((cell, sorted_intervals[best_idx]))
                    used_intervals.add(best_idx)
                else:
                    merges.append(cell)
            for idx, interval in enumerate(sorted_intervals):
                if idx not in used_intervals:
                    splits.append(interval)

        else:
            # One or more MERGE events: each interval claims its best
            # remaining cell; any cell left unclaimed has been merged away.
            used_cells = set()
            for interval in sorted_intervals:
                best_idx, best_overlap = None, 1e-3
                for idx, cell in enumerate(sorted_cells):
                    if idx in used_cells:
                        continue
                    ov = interval_overlap(cell, interval)
                    if ov > best_overlap:
                        best_overlap = ov
                        best_idx = idx
                if best_idx is not None:
                    matched.append((sorted_cells[best_idx], interval))
                    used_cells.add(best_idx)
                else:
                    splits.append(interval)
            for idx, cell in enumerate(sorted_cells):
                if idx not in used_cells:
                    merges.append(cell)

        return matched, splits, merges

    # --------------------------------------------------------------------------
    # Coverage Path Generation inside Monotone Cells
    # --------------------------------------------------------------------------

    def _generate_coverage_path(self, cells: List[Cell]) -> List[Tuple[float, float, float]]:
        """
        Generates lawnmower trajectories inside each cell and sequences them
        via Nearest-Neighbor.

        IMPORTANT: cells produced by this sweep-line BCD are x-monotone -
        each x is guaranteed to intersect the cell in exactly one contiguous
        y-interval. The same is NOT true in the other direction: a fixed y
        can intersect a cell in several disjoint x-segments (e.g. a cell
        that wraps both above and below an obstacle). So coverage passes
        must sweep vertically at fixed x steps, not horizontally at fixed y
        steps - a horizontal sweep can silently "bridge" two segments that
        are actually separated by an obstacle, producing a waypoint list
        that cuts straight through it.
        """
        cell_paths: Dict[int, List[Tuple[float, float, float]]] = {}

        for cell in cells:
            if cell.polygon is None:
                continue

            min_x, min_y, max_x, max_y = cell.polygon.bounds
            x_sweeps = np.arange(min_x + self.row_spacing / 2.0, max_x, self.row_spacing)

            waypoints = []
            reverse = False

            for x in x_sweeps:
                vertical_line = sg.LineString([(x, min_y - 1.0), (x, max_y + 1.0)])
                intersection = cell.polygon.intersection(vertical_line)

                lines = []
                if isinstance(intersection, sg.LineString):
                    if not intersection.is_empty:
                        lines.append(intersection)
                elif isinstance(intersection, sg.MultiLineString):
                    # Should not normally happen for a correctly-built
                    # x-monotone cell, but handle it defensively rather
                    # than silently bridging a gap.
                    lines.extend(list(intersection.geoms))

                for line in lines:
                    x1, y1 = line.coords[0]
                    x2, y2 = line.coords[1]
                    p1 = (x1, y1, self.fly_altitude)
                    p2 = (x2, y2, self.fly_altitude)

                    if reverse:
                        waypoints.extend([p2, p1] if y1 < y2 else [p1, p2])
                    else:
                        waypoints.extend([p1, p2] if y1 < y2 else [p2, p1])

                    reverse = not reverse

            if waypoints:
                cell_paths[cell.cell_id] = waypoints

        # Greedy Nearest-Neighbor Cell Ordering
        ordered_path: List[Tuple[float, float, float]] = []
        unvisited = list(cell_paths.keys())

        if not unvisited:
            return ordered_path

        current_id = unvisited.pop(0)
        ordered_path.extend(cell_paths[current_id])

        while unvisited:
            last_pt = np.array(ordered_path[-1][:2])
            next_id = None
            min_dist = float('inf')

            for candidate_id in unvisited:
                cand_start = np.array(cell_paths[candidate_id][0][:2])
                dist = np.linalg.norm(last_pt - cand_start)
                if dist < min_dist:
                    min_dist = dist
                    next_id = candidate_id

            unvisited.remove(next_id)
            ordered_path.extend(cell_paths[next_id])

        return ordered_path

    # --------------------------------------------------------------------------
    # Output & Artifact Generation
    # --------------------------------------------------------------------------

    def _export_to_csv(self, waypoints: List[Tuple[float, float, float]]):
        output_dir = os.path.expanduser('~/coverage_planner')
        os.makedirs(output_dir, exist_ok=True)
        csv_path = os.path.join(output_dir, 'waypoints.csv')

        with open(csv_path, mode='w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['seq', 'x_local', 'y_local', 'z', 'latitude', 'longitude'])

            for seq, (x, y, z) in enumerate(waypoints):
                lat, lon, _ = self.converter.enu_to_geodetic(x, y, z)
                writer.writerow([seq, f"{x:.3f}", f"{y:.3f}", f"{z:.3f}", f"{lat:.8f}", f"{lon:.8f}"])

        self.get_logger().info(f"Waypoints successfully written to: {csv_path}")

    def _publish_ros_data(self, cells: List[Cell], waypoints: List[Tuple[float, float, float]]):
        # 1. Publish Path Message
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for x, y, z in waypoints:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = path_msg.header.stamp
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.position.z = float(z)
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)

        # 2. Publish Cell Markers
        marker_array = MarkerArray()
        cmap = plt.get_cmap('tab10')

        for i, cell in enumerate(cells):
            if cell.polygon is None:
                continue

            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = path_msg.header.stamp
            marker.ns = 'bcd_cell_boundaries'
            marker.id = cell.cell_id
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.scale.x = 0.3

            color_rgba = cmap(i % 10)
            marker.color = ColorRGBA(r=float(color_rgba[0]), g=float(color_rgba[1]), b=float(color_rgba[2]), a=0.8)

            if cell.polygon.geom_type == 'Polygon':
                polygons = [cell.polygon]
            elif cell.polygon.geom_type == 'MultiPolygon':
                polygons = list(cell.polygon.geoms)
            else:
                polygons = []

            for poly in polygons:
                for x, y in poly.exterior.coords:
                    marker.points.append(Point(x=float(x), y=float(y), z=0.0))

            marker_array.markers.append(marker)

            # Text Label Marker
            text_marker = Marker()
            text_marker.header.frame_id = 'map'
            text_marker.header.stamp = path_msg.header.stamp
            text_marker.ns = 'bcd_cell_labels'
            text_marker.id = cell.cell_id + 1000
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.scale.z = 2.0
            text_marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            text_marker.text = f"Cell {cell.cell_id}"

            centroid = cell.polygon.centroid
            text_marker.pose.position.x = centroid.x
            text_marker.pose.position.y = centroid.y
            text_marker.pose.position.z = 1.0

            marker_array.markers.append(text_marker)

        self.marker_pub.publish(marker_array)
        self.get_logger().info("Published Path and Visualization Markers to ROS2.")

    def _generate_debug_plot(self, cells: List[Cell], waypoints: List[Tuple[float, float, float]]):
        fig, ax = plt.subplots(figsize=(10, 10))

        # Boundary
        bx, by = self.boundary.exterior.xy
        ax.plot(bx, by, 'k-', linewidth=2, label='Boundary')

        # Obstacles
        for i, obs in enumerate(self.obstacles):
            ox, oy = obs.exterior.xy
            ax.fill(ox, oy, color='dimgray', alpha=0.8, label='Obstacle' if i == 0 else "")

        # Cells
        cmap = plt.get_cmap('tab10')
        for i, cell in enumerate(cells):
            if cell.polygon is None:
                continue

            if cell.polygon.geom_type == 'Polygon':
                polygons = [cell.polygon]
            elif cell.polygon.geom_type == 'MultiPolygon':
                polygons = list(cell.polygon.geoms)
            else:
                polygons = []

            for poly in polygons:
                cx, cy = poly.exterior.xy
                ax.fill(cx, cy, color=cmap(i % 10), alpha=0.3)

            centroid = cell.polygon.centroid
            ax.text(centroid.x, centroid.y, f"Cell {cell.cell_id}",
                     fontsize=12, weight='bold', ha='center')

        # Coverage Path
        if waypoints:
            wx = [p[0] for p in waypoints]
            wy = [p[1] for p in waypoints]
            ax.plot(wx, wy, 'r.-', linewidth=1.2, markersize=3, label='Coverage Path')

            ax.plot(wx[0], wy[0], 'go', markersize=8, label='Start')
            ax.plot(wx[-1], wy[-1], 'ro', markersize=8, label='End')

        ax.set_title("Boustrophedon Cellular Decomposition Coverage Plan")
        ax.set_xlabel("X (meters)")
        ax.set_ylabel("Y (meters)")
        ax.grid(True)
        ax.legend(loc='upper right')
        ax.set_aspect('equal')

        output_dir = os.path.expanduser('~/coverage_planner')
        os.makedirs(output_dir, exist_ok=True)
        plot_path = os.path.join(output_dir, 'coverage_plan.png')
        plt.savefig(plot_path, dpi=300, bbox_inches='tight')
        plt.close(fig)
        self.get_logger().info(f"Plot saved successfully to: {plot_path}")


# ==============================================================================
# Main Entry Point
# ==============================================================================

def main(args=None):
    rclpy.init(args=args)
    node = BCDGeometricPlanner()

    node.execute_plan()

    # Process pending callbacks to ensure publication
    rclpy.spin_once(node, timeout_sec=2.0)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
