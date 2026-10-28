#!/usr/bin/env python3
"""
bcd_coverage_multi_node_v2.py
=============================
Obstacle-safe Boustrophedon Cellular Decomposition (BCD) coverage planner for
N drones (default 3). Successor of bcd_coverage_multi_node.py (left untouched).

Why v2 (bug found 28 Oct 2026)
------------------------------
The original node decomposed the map COLUMN by column (vertical slices) but then
swept each cell with HORIZONTAL rows from the left-most to the right-most free
column of that row. At a split/merge it kept extending one branch instead of
closing the cell, so a cell could wrap around an obstacle and its rows ran
straight through it (~18 m of path inside the triangle on coverage_area.pgm).
Transitions between cells were straight lines that were never checked.

What v2 does
------------
 1. Reads the map directly from the map_server YAML/PGM (no ROS needed) with the
    same conventions as nav2 map_server (occupied/free thresholds, origin, y-up).
 2. Inflates obstacles by a real safety distance (default 1.0 m).
 3. Decomposes ROW by row - the same direction as the sweeps - and CLOSES cells at
    every split / merge, so each cell has exactly one free interval per row.
 4. Every sweep row is CLIPPED to that cell's own free interval (never crosses an
    obstacle).
 5. Every connection (row-to-row step, cell-to-cell transition) is checked against
    the inflated map; if blocked it is routed with A* and smoothed.
 6. Cells are ordered greedily, trying all 4 entry corners of each cell.
 7. The path is split into N contiguous pieces of equal ESTIMATED TIME (same model
    as swarm_mission_mavsdk_v4.py: distance / 1.5 m/s + 8 s per corner).
 8. SELF-CHECK: reports metres of path inside obstacles (must be 0), minimum
    clearance, and coverage of the free area; can also check an old plan.

Outputs (NEW folder every run, nothing is overwritten)
------------------------------------------------------
  ~/coverage_map/plans/<date>_<time>_bcd_v2/
      waypoints_drone1.csv ... waypoints_droneN.csv   (seq,x_local_m,y_local_m,z_m)
      waypoints.csv                                   (full ordered path)
      plan_bcd_v2.png                                 (map, cells, paths)
      plan_info.txt                                   (settings + self-check)

Usage
-----
  python3 bcd_coverage_multi_node_v2.py
  python3 bcd_coverage_multi_node_v2.py --map ~/coverage_map/coverage_area.yaml --drones 3
  python3 bcd_coverage_multi_node_v2.py --row-spacing 3 --inflate 1.0 --altitude 10
  python3 bcd_coverage_multi_node_v2.py --check-only ~/coverage_map   # check an existing plan

Requires: numpy, Pillow, PyYAML (optional), matplotlib (for the plot).
"""

import argparse
import csv
import heapq
import math
import os
import time
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

Point = Tuple[float, float]

# ---------------------------------------------------------------- defaults
DEFAULT_MAP = os.path.expanduser("~/coverage_map/coverage_area.yaml")
DEFAULT_OUT_ROOT = os.path.expanduser("~/coverage_map/plans")
ROW_SPACING_M = 3.0
INFLATE_M = 1.0
ALTITUDE_M = 10.0
N_DRONES = 3
EST_SPEED_M_S = 1.5      # same time model as the mission script
TURN_COST_S = 8.0
TURN_ANGLE_DEG = 20.0


# =============================================================================
# Map
# =============================================================================
class GridMap:
    """Occupancy grid with row 0 = lowest y (like nav_msgs/OccupancyGrid)."""

    def __init__(self, occ: np.ndarray, res: float, ox: float, oy: float):
        self.occ = occ.astype(bool)          # True = occupied / unknown
        self.res, self.ox, self.oy = res, ox, oy
        self.h, self.w = occ.shape

    @classmethod
    def from_yaml(cls, yaml_path: str) -> "GridMap":
        from PIL import Image
        meta = _read_yaml(yaml_path)
        img_path = meta["image"]
        if not os.path.isabs(img_path):
            img_path = os.path.join(os.path.dirname(os.path.abspath(yaml_path)), img_path)
        img = np.array(Image.open(img_path).convert("L"), dtype=float)
        p = img / 255.0 if int(meta.get("negate", 0)) else (255.0 - img) / 255.0
        occupied_thresh = float(meta.get("occupied_thresh", 0.65))
        free_thresh = float(meta.get("free_thresh", 0.196))
        free = p < free_thresh
        occ = ~free                                  # occupied AND unknown are not flyable
        _ = occupied_thresh
        occ = occ[::-1, :]                           # image row 0 = top  ->  grid row 0 = bottom
        origin = meta.get("origin", [0.0, 0.0, 0.0])
        return cls(occ, float(meta["resolution"]), float(origin[0]), float(origin[1]))

    def to_world(self, r: float, c: float) -> Point:
        return (self.ox + (c + 0.5) * self.res, self.oy + (r + 0.5) * self.res)

    def to_cell(self, x: float, y: float) -> Tuple[int, int]:
        return (int(math.floor((y - self.oy) / self.res)), int(math.floor((x - self.ox) / self.res)))

    def inside(self, r: int, c: int) -> bool:
        return 0 <= r < self.h and 0 <= c < self.w


def _read_yaml(path: str) -> dict:
    try:
        import yaml
        with open(path) as f:
            return yaml.safe_load(f)
    except ImportError:                              # tiny fallback parser for map yaml files
        meta = {}
        with open(path) as f:
            for line in f:
                if ":" not in line or line.strip().startswith("#"):
                    continue
                k, v = line.split(":", 1)
                v = v.strip()
                if v.startswith("["):
                    meta[k.strip()] = [float(t) for t in v.strip("[]").split(",")]
                else:
                    try:
                        meta[k.strip()] = float(v)
                    except ValueError:
                        meta[k.strip()] = v
        return meta


def inflate(occ: np.ndarray, radius_cells: int) -> np.ndarray:
    """Dilate obstacles with a disk; the map border is treated as an obstacle too."""
    occ = occ.copy()
    occ[0, :] = occ[-1, :] = occ[:, 0] = occ[:, -1] = True
    if radius_cells <= 0:
        return occ
    h, w = occ.shape
    out = occ.copy()
    pad = np.pad(occ, radius_cells, constant_values=True)
    for dr in range(-radius_cells, radius_cells + 1):
        for dc in range(-radius_cells, radius_cells + 1):
            if dr * dr + dc * dc <= radius_cells * radius_cells:
                out |= pad[radius_cells + dr: radius_cells + dr + h,
                           radius_cells + dc: radius_cells + dc + w]
    return out


def clearance_map(occ: np.ndarray, res: float) -> np.ndarray:
    """Approximate Euclidean distance (m) to the nearest occupied cell (3-4 chamfer)."""
    h, w = occ.shape
    INF = 10 ** 9
    d = np.where(occ, 0, INF).astype(np.int64)
    for r in range(h):
        for c in range(w):
            if d[r, c]:
                best = d[r, c]
                if r > 0:
                    best = min(best, d[r - 1, c] + 3)
                    if c > 0:
                        best = min(best, d[r - 1, c - 1] + 4)
                    if c < w - 1:
                        best = min(best, d[r - 1, c + 1] + 4)
                if c > 0:
                    best = min(best, d[r, c - 1] + 3)
                d[r, c] = best
    for r in range(h - 1, -1, -1):
        for c in range(w - 1, -1, -1):
            if d[r, c]:
                best = d[r, c]
                if r < h - 1:
                    best = min(best, d[r + 1, c] + 3)
                    if c > 0:
                        best = min(best, d[r + 1, c - 1] + 4)
                    if c < w - 1:
                        best = min(best, d[r + 1, c + 1] + 4)
                if c < w - 1:
                    best = min(best, d[r, c + 1] + 3)
                d[r, c] = best
    return d / 3.0 * res


# =============================================================================
# Decomposition (row by row, cells closed at every split / merge)
# =============================================================================
class Cell:
    def __init__(self, cid: int):
        self.id = cid
        self.rows: Dict[int, Tuple[int, int]] = {}   # grid row -> (col_start, col_end) inclusive

    @property
    def r0(self) -> int:
        return min(self.rows)

    @property
    def r1(self) -> int:
        return max(self.rows)


def free_intervals(row: np.ndarray) -> List[Tuple[int, int]]:
    out, c, w = [], 0, len(row)
    while c < w:
        if not row[c]:
            s = c
            while c < w and not row[c]:
                c += 1
            out.append((s, c - 1))
        else:
            c += 1
    return out


def decompose(occ_inf: np.ndarray) -> List[Cell]:
    cells: List[Cell] = []
    prev: List[Tuple[int, int, int]] = []            # (c0, c1, cell id) of the previous row
    for r in range(occ_inf.shape[0]):
        cur = free_intervals(occ_inf[r])
        links_cur = [[j for j, (pc0, pc1, _) in enumerate(prev) if min(c1, pc1) >= max(c0, pc0)]
                     for (c0, c1) in cur]
        links_prev = [[i for i, (c0, c1) in enumerate(cur) if min(c1, pc1) >= max(c0, pc0)]
                      for (pc0, pc1, _) in prev]
        new_prev = []
        for i, (c0, c1) in enumerate(cur):
            lp = links_cur[i]
            if len(lp) == 1 and len(links_prev[lp[0]]) == 1:
                cid = prev[lp[0]][2]                 # simple continuation of one cell
            else:
                cid = len(cells)                     # IN / SPLIT / MERGE event -> new cell
                cells.append(Cell(cid))
            cells[cid].rows[r] = (c0, c1)
            new_prev.append((c0, c1, cid))
        prev = new_prev
    return [c for c in cells if c.rows]


# =============================================================================
# Collision checking, A*, smoothing
# =============================================================================
def segment_free(gm: GridMap, occ: np.ndarray, a: Point, b: Point) -> bool:
    n = max(1, int(math.hypot(b[0] - a[0], b[1] - a[1]) / (gm.res * 0.5)))
    for k in range(n + 1):
        r, c = gm.to_cell(a[0] + (b[0] - a[0]) * k / n, a[1] + (b[1] - a[1]) * k / n)
        if not gm.inside(r, c) or occ[r, c]:
            return False
    return True


def astar(gm: GridMap, occ: np.ndarray, a: Point, b: Point) -> Optional[List[Point]]:
    start, goal = gm.to_cell(*a), gm.to_cell(*b)
    for p in (start, goal):
        if not gm.inside(*p) or occ[p]:
            return None
    moves = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
             (1, 1, 1.4142), (1, -1, 1.4142), (-1, 1, 1.4142), (-1, -1, 1.4142)]
    g = {start: 0.0}
    came: Dict[Tuple[int, int], Tuple[int, int]] = {}
    pq = [(0.0, start)]
    while pq:
        _, cur = heapq.heappop(pq)
        if cur == goal:
            break
        for dr, dc, cost in moves:
            nb = (cur[0] + dr, cur[1] + dc)
            if not gm.inside(*nb) or occ[nb]:
                continue
            if dr and dc and (occ[cur[0] + dr, cur[1]] or occ[cur[0], cur[1] + dc]):
                continue                              # no corner cutting
            ng = g[cur] + cost
            if ng < g.get(nb, 1e18):
                g[nb] = ng
                came[nb] = cur
                heapq.heappush(pq, (ng + math.hypot(goal[0] - nb[0], goal[1] - nb[1]), nb))
    if goal not in g:
        return None
    cells = [goal]
    while cells[-1] != start:
        cells.append(came[cells[-1]])
    pts = [gm.to_world(r, c) for r, c in reversed(cells)]
    pts[0], pts[-1] = a, b
    # line-of-sight smoothing
    out, i = [pts[0]], 0
    while i < len(pts) - 1:
        j = len(pts) - 1
        while j > i + 1 and not segment_free(gm, occ, pts[i], pts[j]):
            j -= 1
        out.append(pts[j])
        i = j
    return out


def connect(gm: GridMap, occ: np.ndarray, a: Point, b: Point) -> List[Point]:
    """Points to append after a to reach b without crossing (inflated) obstacles."""
    if segment_free(gm, occ, a, b):
        return [b]
    # try an L-shaped move first (keeps the lawnmower look)
    for mid in ((a[0], b[1]), (b[0], a[1])):
        if segment_free(gm, occ, a, mid) and segment_free(gm, occ, mid, b):
            return [mid, b]
    path = astar(gm, occ, a, b)
    if path is None:
        raise RuntimeError(f"no collision-free connection from {a} to {b}")
    return path[1:]


# =============================================================================
# Coverage path
# =============================================================================
def cell_rows(cell: Cell, spacing_cells: int) -> List[int]:
    r0, r1, half = cell.r0, cell.r1, spacing_cells // 2
    rows = list(range(r0 + half, r1 + 1, spacing_cells))
    if not rows:
        rows = [(r0 + r1) // 2]
    elif r1 - rows[-1] > half:
        rows.append(r1)
    return rows


def cell_path(gm: GridMap, occ: np.ndarray, cell: Cell, rows: List[int],
              from_top: bool, start_left: bool) -> List[Point]:
    order = list(reversed(rows)) if from_top else rows
    pts: List[Point] = []
    left = start_left
    for r in order:
        c0, c1 = cell.rows[r]
        xa, y = gm.to_world(r, c0)
        xb, _ = gm.to_world(r, c1)
        row = [(xa, y), (xb, y)] if left else [(xb, y), (xa, y)]
        if pts:
            pts.extend(connect(gm, occ, pts[-1], row[0])[:-1])
        pts.extend(row if abs(xb - xa) > 1e-6 else row[:1])
        left = not left
    return pts


def plan_coverage(gm: GridMap, occ_inf: np.ndarray, cells: List[Cell], spacing_cells: int,
                  start: Point) -> List[Point]:
    variants = {}
    for cell in cells:
        rows = cell_rows(cell, spacing_cells)
        variants[cell.id] = [cell_path(gm, occ_inf, cell, rows, ft, sl)
                             for ft in (False, True) for sl in (True, False)]
    remaining = {c.id for c in cells}
    path: List[Point] = []
    pos = start
    while remaining:
        best = None
        for cid in remaining:
            for v in variants[cid]:
                if v:
                    d = math.hypot(v[0][0] - pos[0], v[0][1] - pos[1])
                    if best is None or d < best[0]:
                        best = (d, cid, v)
        if best is None:
            break
        _, cid, v = best
        remaining.discard(cid)
        if path:
            path.extend(connect(gm, occ_inf, path[-1], v[0])[:-1])
        path.extend(v)
        pos = path[-1]
    return dedupe(path)


def dedupe(path: List[Point], eps: float = 0.05) -> List[Point]:
    out: List[Point] = []
    for p in path:
        if not out or math.hypot(p[0] - out[-1][0], p[1] - out[-1][1]) > eps:
            out.append(p)
    # drop collinear middle points (straight continuation)
    res: List[Point] = []
    for p in out:
        if len(res) >= 2:
            a, b = res[-2], res[-1]
            cross = (b[0] - a[0]) * (p[1] - b[1]) - (b[1] - a[1]) * (p[0] - b[0])
            dot = (b[0] - a[0]) * (p[0] - b[0]) + (b[1] - a[1]) * (p[1] - b[1])
            if abs(cross) < 1e-6 and dot > 0:
                res[-1] = p
                continue
        res.append(p)
    return res


# =============================================================================
# Time-based split (same model as the mission script)
# =============================================================================
def _turn(a: Point, b: Point, c: Point) -> bool:
    v1, v2 = (b[0] - a[0], b[1] - a[1]), (c[0] - b[0], c[1] - b[1])
    n1, n2 = math.hypot(*v1), math.hypot(*v2)
    if n1 < 1e-6 or n2 < 1e-6:
        return False
    cosang = max(-1.0, min(1.0, (v1[0] * v2[0] + v1[1] * v2[1]) / (n1 * n2)))
    return math.degrees(math.acos(cosang)) > TURN_ANGLE_DEG


def leg_costs(path: List[Point]) -> List[Tuple[float, float]]:
    return [(TURN_COST_S if i > 0 and _turn(path[i - 1], path[i], path[i + 1]) else 0.0,
             math.hypot(path[i + 1][0] - path[i][0], path[i + 1][1] - path[i][1]) / EST_SPEED_M_S)
            for i in range(len(path) - 1)]


def path_time(path: List[Point]) -> float:
    return sum(c + t for c, t in leg_costs(path)) if len(path) > 1 else 0.0


def path_length(path: Sequence[Point]) -> float:
    return sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(path, path[1:]))


def split_by_time(path: List[Point], n: int) -> List[List[Point]]:
    if n <= 1 or len(path) < 2:
        return [path] + [[] for _ in range(n - 1)]
    costs = leg_costs(path)
    total = sum(c + t for c, t in costs)
    targets = [total * k / n for k in range(1, n)]
    segs: List[List[Point]] = [[path[0]]]
    acc, ti = 0.0, 0
    for (a, b), (corner, travel) in zip(zip(path, path[1:]), costs):
        while ti < len(targets) and acc + corner >= targets[ti] - 1e-9:
            segs.append([a])
            ti += 1
        acc += corner
        while ti < len(targets) and acc + travel >= targets[ti] - 1e-9:
            t = (targets[ti] - acc) / travel if travel > 0 else 0.0
            cut = (a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]))
            segs[-1].append(cut)
            segs.append([cut])
            ti += 1
        segs[-1].append(b)
        acc += travel
    while len(segs) < n:
        segs.append([])
    return [dedupe(s) for s in segs]


# =============================================================================
# Self-check
# =============================================================================
def check_paths(gm: GridMap, occ_raw: np.ndarray, occ_inf: np.ndarray, clear: np.ndarray,
                paths: List[List[Point]], spacing_m: float) -> dict:
    inside_m = inside_inf_m = 0.0
    min_clear = float("inf")
    covered = np.zeros_like(occ_raw, dtype=bool)
    half_cells = int(round(spacing_m / 2 / gm.res))
    step = gm.res * 0.5
    for path in paths:
        for a, b in zip(path, path[1:]):
            L = math.hypot(b[0] - a[0], b[1] - a[1])
            n = max(1, int(L / step))
            horizontal = abs(b[1] - a[1]) < 0.3
            for k in range(n + 1):
                x, y = a[0] + (b[0] - a[0]) * k / n, a[1] + (b[1] - a[1]) * k / n
                r, c = gm.to_cell(x, y)
                if not gm.inside(r, c):
                    inside_m += L / (n + 1)
                    continue
                if occ_raw[r, c]:
                    inside_m += L / (n + 1)
                if occ_inf[r, c]:
                    inside_inf_m += L / (n + 1)
                min_clear = min(min_clear, clear[r, c])
                if horizontal:                           # sweep rows see a swath
                    covered[max(0, r - half_cells): r + half_cells + 1, c] = True
    free = ~occ_raw
    free[0, :] = free[-1, :] = free[:, 0] = free[:, -1] = False
    flyable = free & ~occ_inf
    return {"inside_obstacles_m": inside_m, "inside_inflated_m": inside_inf_m,
            "min_clearance_m": min_clear,
            "free_area_m2": float(free.sum()) * gm.res ** 2,
            "covered_free_m2": float((covered & free).sum()) * gm.res ** 2,
            "flyable_area_m2": float(flyable.sum()) * gm.res ** 2,
            "covered_flyable_m2": float((covered & flyable).sum()) * gm.res ** 2}


# =============================================================================
# Output
# =============================================================================
def make_out_dir(root: str) -> str:
    os.makedirs(root, exist_ok=True)
    stamp = time.strftime("%Y-%m-%d_%H-%M-%S")
    for k in range(1000):
        p = os.path.join(root, f"{stamp}_bcd_v2" + (f"_{k}" if k else ""))
        try:
            os.makedirs(p)
            return p
        except FileExistsError:
            continue
    raise RuntimeError("could not create output folder")


def write_csv(path: str, pts: List[Point], z: float):
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["seq", "x_local_m", "y_local_m", "z_m"])
        for i, (x, y) in enumerate(pts):
            w.writerow([i, f"{x:.3f}", f"{y:.3f}", f"{z:.1f}"])


def read_csv(path: str) -> List[Point]:
    with open(path, newline="") as f:
        return [(float(r.get("x_local_m", r.get("x_local"))), float(r.get("y_local_m", r.get("y_local"))))
                for r in csv.DictReader(f)]


def plot(gm: GridMap, occ_raw, occ_inf, cells, pieces, out_png, title):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib not installed - no plot")
        return
    ext = [gm.ox, gm.ox + gm.w * gm.res, gm.oy, gm.oy + gm.h * gm.res]
    img = np.ones((gm.h, gm.w, 3))
    img[occ_inf & ~occ_raw] = (0.85, 0.85, 0.85)          # safety margin
    img[occ_raw] = (0.15, 0.15, 0.15)                     # obstacles
    fig, ax = plt.subplots(figsize=(9, 9))
    ax.imshow(img, extent=ext, origin="lower")
    if cells:
        cmap = plt.get_cmap("tab20")
        for k, cell in enumerate(cells):
            for r, (c0, c1) in cell.rows.items():
                ax.add_patch(plt.Rectangle((gm.ox + c0 * gm.res, gm.oy + r * gm.res),
                                           (c1 - c0 + 1) * gm.res, gm.res,
                                           color=cmap(k % 20), alpha=0.18, lw=0))
            cx, cy = gm.to_world((cell.r0 + cell.r1) / 2, np.mean(cell.rows[(cell.r0 + cell.r1) // 2]))
            ax.text(cx, cy, str(k), fontsize=8, ha="center", va="center", color="gray")
    colors = ["tab:blue", "tab:orange", "tab:green", "tab:red", "tab:purple"]
    for i, p in enumerate(pieces):
        if len(p) > 1:
            col = colors[i % len(colors)]
            for a, b in zip(p, p[1:]):
                sweep = abs(b[1] - a[1]) < 0.3 or (abs(b[0] - a[0]) < 0.3 and abs(b[1] - a[1]) <= ROW_SPACING_M + 0.5)
                ax.plot([a[0], b[0]], [a[1], b[1]], "-" if sweep else "--", color=col, lw=1.4)
            ax.plot(p[0][0], p[0][1], "^", color=col, ms=9,
                    label=f"Drone {i + 1}: {path_length(p):.0f} m, est. {path_time(p) / 60:.1f} min")
    ax.set_xlabel("East / map x (m)")
    ax.set_ylabel("North / map y (m)")
    ax.set_title(title, fontsize=10)
    ax.legend(fontsize=8, loc="lower right")
    fig.savefig(out_png, dpi=140, bbox_inches="tight")
    plt.close(fig)


# =============================================================================
# Main
# =============================================================================
def main():
    ap = argparse.ArgumentParser(description="Obstacle-safe BCD coverage planner (v2)")
    ap.add_argument("--map", default=DEFAULT_MAP, help="map_server YAML file")
    ap.add_argument("--drones", type=int, default=N_DRONES)
    ap.add_argument("--row-spacing", type=float, default=ROW_SPACING_M)
    ap.add_argument("--inflate", type=float, default=INFLATE_M, help="safety distance to obstacles (m)")
    ap.add_argument("--altitude", type=float, default=ALTITUDE_M, help="z written into the CSVs")
    ap.add_argument("--start", type=float, nargs=2, default=(0.0, 0.0), metavar=("X", "Y"),
                    help="where the drones start (map frame), used to order the cells")
    ap.add_argument("--out-root", default=DEFAULT_OUT_ROOT)
    ap.add_argument("--check-only", metavar="DIR",
                    help="only check an existing plan (waypoints_drone*.csv in DIR) against the map")
    args = ap.parse_args()

    gm = GridMap.from_yaml(os.path.expanduser(args.map))
    occ_raw = gm.occ.copy()
    occ_raw_b = occ_raw.copy()
    occ_raw_b[0, :] = occ_raw_b[-1, :] = occ_raw_b[:, 0] = occ_raw_b[:, -1] = False
    rad = max(1, int(math.ceil(args.inflate / gm.res)))
    occ_inf = inflate(occ_raw, rad)
    clear = clearance_map(occ_raw_b | np.pad(np.zeros((gm.h - 2, gm.w - 2), bool), 1, constant_values=True), gm.res)
    print(f"map {gm.w}x{gm.h} @ {gm.res} m, origin ({gm.ox}, {gm.oy}); inflation {rad * gm.res:.1f} m")

    if args.check_only:
        d = os.path.expanduser(args.check_only)
        pieces = [read_csv(os.path.join(d, f)) for f in sorted(os.listdir(d))
                  if f.startswith("waypoints_drone") and f.endswith(".csv")]
        chk = check_paths(gm, occ_raw_b, occ_inf, clear, pieces, args.row_spacing)
        print(f"checked {len(pieces)} files in {d}:")
        print(f"  path inside obstacles      : {chk['inside_obstacles_m']:.1f} m")
        print(f"  path inside safety margin  : {chk['inside_inflated_m']:.1f} m")
        print(f"  minimum clearance          : {chk['min_clearance_m']:.2f} m")
        print(f"  free area covered (sweeps) : {100 * chk['covered_free_m2'] / chk['free_area_m2']:.1f} %")
        print(f"  flyable area covered       : {100 * chk['covered_flyable_m2'] / chk['flyable_area_m2']:.1f} %"
              f"  (flyable = free and at least {args.inflate:.1f} m from obstacles)")
        return

    spacing_cells = max(1, int(round(args.row_spacing / gm.res)))
    t0 = time.time()
    cells = decompose(occ_inf)
    path = plan_coverage(gm, occ_inf, cells, spacing_cells, tuple(args.start))
    pieces = split_by_time(path, args.drones)
    chk = check_paths(gm, occ_raw_b, occ_inf, clear, pieces, args.row_spacing)
    dt = time.time() - t0

    out = make_out_dir(os.path.expanduser(args.out_root))
    write_csv(os.path.join(out, "waypoints.csv"), path, args.altitude)
    for i, p in enumerate(pieces, start=1):
        write_csv(os.path.join(out, f"waypoints_drone{i}.csv"), p, args.altitude)
    cov_pct = 100 * chk["covered_free_m2"] / chk["free_area_m2"] if chk["free_area_m2"] else 0.0
    lines = [
        "bcd_coverage_multi_node_v2.py",
        f"created          : {time.strftime('%Y-%m-%d %H:%M:%S')}",
        f"map              : {os.path.expanduser(args.map)}",
        f"settings         : drones={args.drones} row_spacing={args.row_spacing} m "
        f"inflate={rad * gm.res:.1f} m altitude={args.altitude} m start={tuple(args.start)}",
        f"cells            : {len(cells)}",
        f"full path        : {len(path)} waypoints, {path_length(path):.1f} m, "
        f"est. {path_time(path) / 60:.1f} min for one drone",
    ]
    for i, p in enumerate(pieces, start=1):
        lines.append(f"drone {i}          : {len(p)} waypoints, {path_length(p):.1f} m, "
                     f"est. {path_time(p) / 60:.2f} min")
    lines += [
        "SELF-CHECK",
        f"  path inside obstacles     : {chk['inside_obstacles_m']:.2f} m   (must be 0)",
        f"  path inside safety margin : {chk['inside_inflated_m']:.2f} m",
        f"  minimum clearance         : {chk['min_clearance_m']:.2f} m",
        f"  free area covered (sweeps): {cov_pct:.1f} %  of {chk['free_area_m2']:.0f} m2 "
        f"(strips within the {rad * gm.res:.1f} m safety margin are not flown)",
        f"  flyable area covered      : {100 * chk['covered_flyable_m2'] / chk['flyable_area_m2']:.1f} %  of "
        f"{chk['flyable_area_m2']:.0f} m2 (area at least {rad * gm.res:.1f} m from obstacles)",
        f"planning time    : {dt:.1f} s",
        f"output folder    : {out}",
    ]
    with open(os.path.join(out, "plan_info.txt"), "w") as f:
        f.write("\n".join(lines) + "\n")
    print("\n".join(lines))
    plot(gm, occ_raw_b, occ_inf, cells, pieces, os.path.join(out, "plan_bcd_v2.png"),
         f"BCD v2 plan: {len(cells)} cells, {path_length(path):.0f} m, "
         f"{chk['inside_obstacles_m']:.1f} m inside obstacles, clearance >= {chk['min_clearance_m']:.2f} m\n"
         "(dark = obstacle, light grey = safety margin, dashed = transit)")
    print(f"plot -> {os.path.join(out, 'plan_bcd_v2.png')}")


if __name__ == "__main__":
    main()
