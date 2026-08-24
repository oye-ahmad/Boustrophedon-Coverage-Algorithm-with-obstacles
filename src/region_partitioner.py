"""
region_partitioner.py

Splits the OUTPUT of bcd_coverage_node.py's own greedy_sequence() /
cell_paths into 3 contiguous, workload-balanced regions - one per drone.

Works directly against the real data structures already produced by
BCDCoverageNode.plan_full_path():
  - cell_paths: Dict[int, List[Tuple[float, float]]]   (cell_id -> lawnmower path)
  - ordered_ids: List[int]                              (greedy_sequence() output)

No re-decomposition or re-sequencing needed: cutting the existing
nearest-neighbor sweep into 3 contiguous pieces gives 3 spatially
separated regions "for free".
"""

import math
from typing import Dict, List, Tuple


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
    path length (using the SAME lawnmower paths bcd_coverage_node.py
    already generated per cell). Returns a list of n_drones cell-id lists.
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
    Same stitching logic as BCDCoverageNode.plan_full_path(): concatenate
    a region's cell paths in order, inserting a straight-line transition
    between the end of one cell and the start of the next.
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


def region_summary(regions: List[List[int]], cell_paths: Dict[int, List[Tuple[float, float]]]) -> None:
    for i, region in enumerate(regions, start=1):
        total_len = sum(_path_length(cell_paths[cid]) for cid in region)
        print(f"Drone {i}: {len(region)} cells {region}, total path length ~{total_len:.1f} m")
