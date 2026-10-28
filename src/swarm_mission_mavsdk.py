#!/usr/bin/env python3
"""
swarm_mission_mavsdk.py
=======================
Multi-drone (default 3) Boustrophedon coverage mission for ArduPilot SITL
(GUIDED mode) using MAVSDK-Python offboard position setpoints.

This is the cleaned-up successor of multi_drone_offboard_mission.py (that
file is left untouched). It reads the waypoint CSVs written by
bcd_coverage_multi_node.py (or bcd_coverage_node.py) from ~/coverage_map/.

What was fixed / improved
-------------------------
 1. AXES      Map/ROS frame is ENU: x_local = EAST, y_local = NORTH.
              The old script sent north=x, east=y, which mirrored every path
              across the diagonal (invisible in a square empty map, dangerous
              with obstacles).
 2. FRAME     Each ArduPilot instance has its OWN local NED origin. At start-up
              the script measures every drone's origin and converts it into ONE
              shared map frame, so map point (x, y) is the same spot on the
              ground for every drone even if they spawn at different places.
 3. SPLIT     The whole coverage path is re-split by PATH LENGTH into equal
              pieces (cutting inside cells if needed). The old planner split by
              cell COUNT, so with fewer cells than drones (e.g. an obstacle-free
              map -> 1 cell) drones 2 and 3 got no waypoints at all.
              Pieces are assigned to the drones whose homes are closest, and a
              piece is flown in reverse if that end is nearer.
 4. LAYERS    Each drone cruises at its own altitude (default 8 / 11 / 14 m),
              which removes almost all conflicts by design.
 5. AVOIDANCE 3-D check that predicts the closest approach from velocities
              (horizontal AND vertical), not just the current distance.
              The lower-priority drone HOLDS if the other will pass by, or
              EVADES (moves vertically away, or sideways from a drone that is
              taking off / landing) if holding would not help. A drone that
              is taking off / landing always has right of way. Decisions are
              kept until the other drone has really passed (no hold/resume
              flicker); a hold that lasts too long escalates to evade.
 6. RETURN    Drones fly back to their own home at their own layer altitude,
              wait until nobody is flying below them, then land (ArduPilot RTL
              would send them all to the same RTL_ALT).
 7. ROBUST    Timeouts on connect / health / climb / every waypoint / landing,
              arming retries, no silent `except: pass`, per-drone battery
              failsafe, Ctrl+C makes every drone land, setpoints are streamed by
              a dedicated 10 Hz task, telemetry streams auto-restart.
 8. OUTPUT    actual_path_droneN.csv (same first columns as before, plus
              map-frame columns), flown_waypoints_droneN.csv, mission_summary.txt
              and swarm_planned_vs_actual.png, all in ~/coverage_map/.

Usage
-----
  python3 swarm_mission_mavsdk.py --dry-run        # plan + plot only, no drones
  python3 swarm_mission_mavsdk.py                  # fly all drones in DRONES
  python3 swarm_mission_mavsdk.py --num-drones 2   # use only the first 2
  python3 swarm_mission_mavsdk.py --split planner  # keep the planner's split
  python3 swarm_mission_mavsdk.py --no-layers      # everyone at the CSV altitude
  python3 swarm_mission_mavsdk.py --plot-only      # re-plot the last mission

Requires: mavsdk (2.x, gRPC API: System(port=...)), optional matplotlib.
"""

import argparse
import asyncio
import csv
import itertools
import math
import os
import signal
import time
from typing import Dict, List, Optional, Sequence, Tuple

# =============================================================================
# CONFIGURATION  (edit here)
# =============================================================================
WAYPOINT_DIR = os.path.expanduser("~/coverage_map")

# One entry per drone. Lower id = higher priority for collision avoidance.
# altitude_m = cruise altitude (layer) used for transit, coverage and return.
DRONES = [
    {"id": 1, "url": "udpin://0.0.0.0:14550", "grpc_port": 50051, "altitude_m": 8.0},
    {"id": 2, "url": "udpin://0.0.0.0:14560", "grpc_port": 50052, "altitude_m": 11.0},
    {"id": 3, "url": "udpin://0.0.0.0:14570", "grpc_port": 50053, "altitude_m": 14.0},
]

# Geographic position of map point (0, 0). None = use drone 1's local origin
# (this matches how the old scripts behaved).
MAP_ORIGIN_LATLON: Optional[Tuple[float, float]] = None

# Flight
ACCEPTANCE_RADIUS_M = 1.0     # waypoint reached when closer than this (3-D)
HOME_ACCEPTANCE_M = 1.5
CRUISE_SPEED_M_S = 3.0        # requested max speed (best effort) + timeout estimate
YAW_DEG = 0.0                 # constant heading -> constant camera footprint
SETPOINT_RATE_HZ = 10.0
CONTROL_PERIOD_S = 0.1
MAX_ALTITUDE_M = 30.0
ARM_RETRIES = 3

# Timeouts
CONNECT_TIMEOUT_S = 60.0
HEALTH_TIMEOUT_S = 90.0
FRAME_TIMEOUT_S = 20.0
CLIMB_TIMEOUT_S = 60.0
LAND_TIMEOUT_S = 120.0
WAYPOINT_TIMEOUT_MIN_S = 30.0

# Collision avoidance
SAFE_DIST_M = 5.0             # horizontal separation we want to keep
EMERGENCY_DIST_M = 2.5        # closer than this -> climb away
VERTICAL_SAFE_M = 2.0         # vertical separation that counts as "safe"
PREDICT_HORIZON_S = 3.0       # look-ahead for predicted closest approach
CLEAR_HYSTERESIS_M = 1.0      # must open up this much extra before resuming
HOLD_TIMEOUT_S = 20.0         # hold longer than this -> escalate to evade
EVADE_CLIMB_M = 4.0           # vertical move when evading (away from the other drone)
MIN_EVADE_ALT_M = 3.0
EVADE_PUSH_M = 2.0
STAGGER_TAKEOFF_S = 8.0       # used only if two homes are closer than SAFE_DIST_M

# Failsafe / logging
BATTERY_FAILSAFE_PCT = 25.0
LOG_RATE_HZ = 5.0
TELEMETRY_RATE_HZ = 10.0

R_EARTH = 6378137.0
Point = Tuple[float, float, float]          # (east/x, north/y, alt/z) in map frame


# =============================================================================
# Small helpers
# =============================================================================
_T0 = time.monotonic()


def log(tag: str, msg: str) -> None:
    print(f"[{time.monotonic() - _T0:7.1f}s] {tag:<8} {msg}", flush=True)


def latlon_to_en(lat: float, lon: float, lat0: float, lon0: float) -> Tuple[float, float]:
    """Flat-earth lat/lon -> (east, north) metres relative to (lat0, lon0)."""
    e = math.radians(lon - lon0) * R_EARTH * math.cos(math.radians(lat0))
    n = math.radians(lat - lat0) * R_EARTH
    return e, n


def en_to_latlon(e: float, n: float, lat0: float, lon0: float) -> Tuple[float, float]:
    lat = lat0 + math.degrees(n / R_EARTH)
    lon = lon0 + math.degrees(e / (R_EARTH * math.cos(math.radians(lat0))))
    return lat, lon


def dist2(a: Sequence[float], b: Sequence[float]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def dist3(a: Sequence[float], b: Sequence[float]) -> float:
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2)


def path_length(path: Sequence[Sequence[float]]) -> float:
    return sum(dist2(a, b) for a, b in zip(path, path[1:]))


def dedupe(path: List[Point], eps: float = 0.05) -> List[Point]:
    out: List[Point] = []
    for p in path:
        if not out or dist2(out[-1], p) > eps:
            out.append(p)
    return out


# =============================================================================
# Waypoints: loading, splitting, assignment
# =============================================================================
def read_waypoints(path: str) -> Optional[List[Point]]:
    """Read a planner CSV. Accepts x_local_m/x_local, y_local_m/y_local, z_m/z.
    Returns None if the file does not exist, [] if it is empty."""
    if not os.path.exists(path):
        return None
    pts: List[Point] = []
    with open(path, newline="") as f:
        for row in csv.DictReader(f):
            try:
                x = float(row.get("x_local_m", row.get("x_local")))
                y = float(row.get("y_local_m", row.get("y_local")))
                z = float(row.get("z_m", row.get("z", 10.0)))
            except (TypeError, ValueError) as e:
                raise ValueError(f"{path}: bad row {row} ({e})") from e
            pts.append((x, y, z))
    return pts


def split_by_length(path: List[Point], n: int) -> List[List[Point]]:
    """Cut one ordered path into n contiguous pieces of equal length."""
    path = dedupe(path)
    if n <= 1 or len(path) < 2:
        return [path] + [[] for _ in range(n - 1)]
    total = path_length(path)
    targets = [total * k / n for k in range(1, n)]
    segs: List[List[Point]] = [[path[0]]]
    acc, ti = 0.0, 0
    for a, b in zip(path, path[1:]):
        seg_len = dist2(a, b)
        while ti < len(targets) and acc + seg_len >= targets[ti] - 1e-9:
            t = (targets[ti] - acc) / seg_len if seg_len > 0 else 0.0
            cut = tuple(a[i] + t * (b[i] - a[i]) for i in range(3))
            segs[-1].append(cut)
            segs.append([cut])
            ti += 1
        segs[-1].append(b)
        acc += seg_len
    return [dedupe(s) for s in segs]


def load_plan(wdir: str, n: int, mode: str) -> Tuple[List[List[Point]], str]:
    """Return (n segments, description of how they were made)."""
    per = [read_waypoints(os.path.join(wdir, f"waypoints_drone{i}.csv")) for i in range(1, n + 1)]
    have_all = all(p for p in per)

    if mode == "planner":
        if not have_all:
            raise SystemExit("--split planner: some waypoints_droneN.csv are missing or empty.")
        return [dedupe(p) for p in per], "planner split (as written by bcd_coverage_multi_node)"

    if mode == "auto" and have_all:
        lens = [path_length(p) for p in per]
        if min(lens) > 0 and max(lens) / min(lens) <= 1.25:
            return [dedupe(p) for p in per], "planner split (already balanced within 25%)"

    full: List[Point] = []
    for p in per:
        if p:
            full.extend(p)
    source = "waypoints_drone*.csv"
    if not full:
        single = read_waypoints(os.path.join(wdir, "waypoints.csv"))
        if not single:
            raise SystemExit(f"No waypoints found in {wdir} "
                             "(need waypoints_droneN.csv or waypoints.csv).")
        full, source = single, "waypoints.csv"
    return split_by_length(full, n), f"re-split by path length from {source}"


def assign_segments(segments: List[List[Point]],
                    homes: List[Tuple[float, float]]) -> List[List[Point]]:
    """Give each drone one segment (possibly reversed) so the longest
    home->start transit is as short as possible (then total transit)."""
    n = len(homes)
    best, best_cost = None, None
    for perm in itertools.permutations(range(n)):
        for flips in itertools.product((False, True), repeat=n):
            transits, starts = [], []
            for d in range(n):
                seg = segments[perm[d]]
                if not seg:
                    transits.append(0.0)
                    continue
                start = seg[-1] if flips[d] else seg[0]
                starts.append(start)
                transits.append(dist2(homes[d], start))
            # never let two drones begin at the same spot (e.g. both ends of one cut)
            clashes = sum(1 for a, b in itertools.combinations(starts, 2)
                          if dist2(a, b) < 2 * SAFE_DIST_M)
            cost = (clashes, max(transits), sum(transits))
            if best_cost is None or cost < best_cost:
                best_cost, best = cost, (perm, flips)
    perm, flips = best
    return [list(reversed(segments[perm[d]])) if flips[d] else list(segments[perm[d]])
            for d in range(n)]


# =============================================================================
# One drone
# =============================================================================
class Drone:
    def __init__(self, cfg: dict, swarm: "Swarm", system_factory):
        self.id: int = cfg["id"]
        self.url: str = cfg["url"]
        self.grpc_port: int = cfg["grpc_port"]
        self.alt: float = min(float(cfg["altitude_m"]), MAX_ALTITUDE_M)
        self.tag = f"Drone {self.id}"
        self.swarm = swarm
        self._factory = system_factory
        self.sys = None

        # telemetry
        self.lat = self.lon = self.abs_alt = None
        self.rel_alt = 0.0
        self.ned: Optional[Tuple[float, float, float]] = None   # n, e, d
        self.vel = (0.0, 0.0, 0.0)                              # vn, ve, vd
        self.battery_pct: Optional[float] = None
        self.armed = False

        # frames
        self.origin_latlon: Optional[Tuple[float, float]] = None
        self.off_e = self.off_n = 0.0          # this drone's NED origin in map frame
        self.frame_ready = False
        self.home_map: Optional[Point] = None

        # control
        self.goal: Optional[Point] = None
        self.override: Optional[str] = None    # None | "hold" | "evade"
        self.override_point: Optional[Point] = None
        self.override_from: Optional[int] = None
        self.override_since = 0.0
        self.resume_speed = 0.0                # speed we had when we last yielded
        self.streaming = False
        self.abort_reason: Optional[str] = None
        self.status = "init"

        # metrics
        self.waypoints: List[Point] = []
        self.reached = 0
        self.missed: List[int] = []
        self.hold_time = 0.0
        self.holds = self.evades = 0
        self.flown_m = 0.0
        self.t_takeoff = self.t_landed = None

        self._tasks: List[asyncio.Task] = []
        self._stream_task: Optional[asyncio.Task] = None

    # ---------------------------------------------------------------- logging
    def info(self, m):
        log(self.tag, m)

    def warn(self, m):
        log(self.tag, "WARNING: " + m)

    # ------------------------------------------------------------- geometry
    @property
    def airborne(self) -> bool:
        return self.armed and self.rel_alt > 1.0

    def map_pos(self) -> Point:
        n, e, d = self.ned
        return (e + self.off_e, n + self.off_n, -d)

    def map_vel(self) -> Point:
        vn, ve, vd = self.vel
        return (ve, vn, -vd)

    def intended_vel(self) -> Tuple[float, float]:
        """Horizontal velocity towards the current goal at cruise speed."""
        if self.goal is None:
            return (0.0, 0.0)
        p = self.map_pos()
        dx, dy = self.goal[0] - p[0], self.goal[1] - p[1]
        d = math.hypot(dx, dy)
        v = max(CRUISE_SPEED_M_S, self.resume_speed)
        return (0.0, 0.0) if d < 0.5 else (v * dx / d, v * dy / d)

    def to_ned(self, p: Point):
        from mavsdk.offboard import PositionNedYaw
        return PositionNedYaw(north_m=p[1] - self.off_n, east_m=p[0] - self.off_e,
                              down_m=-p[2], yaw_deg=YAW_DEG)

    # ----------------------------------------------------------- connection
    async def init_system(self):
        self.sys = self._factory(self.grpc_port)

    async def connect(self) -> bool:
        self.info(f"connecting on {self.url} (gRPC {self.grpc_port}) ...")
        try:
            await self.sys.connect(system_address=self.url)

            async def _connected():
                async for s in self.sys.core.connection_state():
                    if s.is_connected:
                        return

            await asyncio.wait_for(_connected(), CONNECT_TIMEOUT_S)
            self.info("connected, waiting for position lock / armable ...")

            async def _healthy():
                async for h in self.sys.telemetry.health():
                    if (h.is_global_position_ok or h.is_local_position_ok) and h.is_armable:
                        return

            await asyncio.wait_for(_healthy(), HEALTH_TIMEOUT_S)
            self.info("health OK")
            return True
        except asyncio.TimeoutError:
            self.warn("timed out during connect / health check")
        except Exception as e:  # noqa: BLE001  (report anything, never hide it)
            self.warn(f"connect failed: {e!r}")
        return False

    # ------------------------------------------------------------ telemetry
    def start_telemetry(self):
        t = self.sys.telemetry
        self._tasks += [
            asyncio.create_task(self._watch("position", t.position, self._on_position)),
            asyncio.create_task(self._watch("pos/vel NED", t.position_velocity_ned, self._on_ned)),
            asyncio.create_task(self._watch("battery", t.battery, self._on_battery)),
            asyncio.create_task(self._watch("armed", t.armed, self._on_armed)),
            asyncio.create_task(self._log_loop()),
        ]

    async def set_rates(self):
        t = self.sys.telemetry
        for name, fn in (("position", t.set_rate_position),
                         ("position_velocity_ned", t.set_rate_position_velocity_ned)):
            try:
                await fn(TELEMETRY_RATE_HZ)
            except Exception as e:  # noqa: BLE001
                self.warn(f"could not set {name} rate ({e!r}); using default rate")

    async def _watch(self, name, stream_fn, handler):
        while True:
            try:
                async for msg in stream_fn():
                    handler(msg)
            except asyncio.CancelledError:
                raise
            except Exception as e:  # noqa: BLE001
                self.warn(f"telemetry '{name}' stream error {e!r}; restarting in 1 s")
            await asyncio.sleep(1.0)

    def _on_position(self, p):
        self.lat, self.lon = p.latitude_deg, p.longitude_deg
        self.abs_alt, self.rel_alt = p.absolute_altitude_m, p.relative_altitude_m

    def _on_ned(self, pv):
        new = (pv.position.north_m, pv.position.east_m, pv.position.down_m)
        if self.ned is not None and self.airborne:
            self.flown_m += math.hypot(new[0] - self.ned[0], new[1] - self.ned[1])
        self.ned = new
        self.vel = (pv.velocity.north_m_s, pv.velocity.east_m_s, pv.velocity.down_m_s)

    def _on_battery(self, b):
        pct = b.remaining_percent
        if pct is None or math.isnan(pct) or pct < 0:
            return
        if pct <= 1.0:                     # MAVSDK 1.x reports 0..1
            pct *= 100.0
        self.battery_pct = pct
        if pct < BATTERY_FAILSAFE_PCT and self.airborne and self.abort_reason is None:
            self.abort_reason = f"battery {pct:.0f}% < {BATTERY_FAILSAFE_PCT:.0f}%"
            self.warn(f"FAILSAFE: {self.abort_reason} -> returning home")

    def _on_armed(self, a):
        self.armed = bool(a)

    async def _log_loop(self):
        os.makedirs(WAYPOINT_DIR, exist_ok=True)
        path = os.path.join(WAYPOINT_DIR, f"actual_path_drone{self.id}.csv")
        with open(path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["timestamp", "latitude", "longitude", "abs_altitude", "rel_altitude",
                        "map_e", "map_n", "map_alt", "status", "override", "battery_pct"])
            while True:
                if self.lat is not None and self.frame_ready and self.ned is not None \
                        and self.status not in ("init", "ready", "done"):
                    e, n, a = self.map_pos()
                    w.writerow([f"{time.time():.3f}", f"{self.lat:.8f}", f"{self.lon:.8f}",
                                f"{self.abs_alt:.2f}", f"{self.rel_alt:.2f}",
                                f"{e:.2f}", f"{n:.2f}", f"{a:.2f}", self.status,
                                self.override or "", "" if self.battery_pct is None
                                else f"{self.battery_pct:.0f}"])
                    f.flush()
                await asyncio.sleep(1.0 / LOG_RATE_HZ)

    # --------------------------------------------------------------- frames
    async def measure_origin(self):
        """Find where this drone's local NED origin is on the globe:
        origin = current global position shifted back by the current NED offset."""
        t_end = time.monotonic() + FRAME_TIMEOUT_S
        while self.lat is None or self.ned is None:
            if time.monotonic() > t_end:
                raise RuntimeError(f"{self.tag}: no position / NED telemetry")
            await asyncio.sleep(0.2)
        n, e, _ = self.ned
        self.origin_latlon = en_to_latlon(-e, -n, self.lat, self.lon)

    def set_map_origin(self, lat0: float, lon0: float):
        self.off_e, self.off_n = latlon_to_en(*self.origin_latlon, lat0, lon0)
        self.frame_ready = True
        self.home_map = self.map_pos()

    # ------------------------------------------------------- setpoint stream
    def _current_target(self) -> Optional[Point]:
        return self.override_point if self.override else self.goal

    async def _stream_setpoints(self):
        failures = 0
        while self.streaming:
            tgt = self._current_target()
            if tgt is not None:
                try:
                    await self.sys.offboard.set_position_ned(self.to_ned(tgt))
                    failures = 0
                except Exception as e:  # noqa: BLE001
                    failures += 1
                    if failures in (1, 10) or failures % 50 == 0:
                        self.warn(f"setpoint send failed ({failures}x in a row): {e!r}")
            await asyncio.sleep(1.0 / SETPOINT_RATE_HZ)

    # ------------------------------------------------------------- movement
    async def go_to(self, p: Point, accept: float, label: str) -> bool:
        """Fly to map point p. Returns True when reached, False on timeout /
        abort / stop. Time spent holding for other drones does not count
        against the timeout."""
        self.goal = p
        budget = max(WAYPOINT_TIMEOUT_MIN_S,
                     3.0 * dist3(self.map_pos(), p) / CRUISE_SPEED_M_S + 10.0)
        active, last = 0.0, time.monotonic()
        while True:
            if self.swarm.stop_event.is_set() or (self.abort_reason and self.status == "coverage"):
                return False
            now = time.monotonic()
            dt, last = now - last, now
            if self.override:
                self.hold_time += dt
            else:
                active += dt
                d = dist3(self.map_pos(), p)
                if d <= accept:
                    return True
                if active > budget:
                    self.warn(f"timeout reaching {label} (still {d:.1f} m away) - skipping it")
                    return False
            await asyncio.sleep(CONTROL_PERIOD_S)

    async def _takeoff(self):
        if self.swarm.stop_event.is_set():
            raise RuntimeError("stop requested before takeoff")
        self.status = "takeoff"
        await self.sys.action.set_takeoff_altitude(self.alt)
        try:
            await self.sys.action.set_maximum_speed(CRUISE_SPEED_M_S)
        except Exception as e:  # noqa: BLE001
            self.warn(f"set_maximum_speed not accepted ({e!r}); using autopilot default")

        for attempt in range(1, ARM_RETRIES + 1):
            try:
                await self.sys.action.arm()
                break
            except Exception as e:  # noqa: BLE001
                self.warn(f"arming attempt {attempt}/{ARM_RETRIES} failed: {e!r}")
                if attempt == ARM_RETRIES:
                    raise RuntimeError("could not arm") from e
                await asyncio.sleep(3.0)
        self.info(f"armed, taking off to {self.alt:.1f} m")
        await self.sys.action.takeoff()
        self.t_takeoff = time.monotonic()

        t_end = time.monotonic() + CLIMB_TIMEOUT_S
        while self.rel_alt < self.alt - 0.5:
            if self.swarm.stop_event.is_set():
                raise RuntimeError("stop requested during climb")
            if time.monotonic() > t_end:
                raise RuntimeError(f"climb timeout (at {self.rel_alt:.1f} m of {self.alt:.1f} m)")
            await asyncio.sleep(0.2)
        self.info(f"airborne at {self.rel_alt:.1f} m")

    async def _start_offboard(self):
        from mavsdk.offboard import OffboardError
        e, n, _ = self.map_pos()
        self.goal = (e, n, self.alt)                 # hold here first
        await self.sys.offboard.set_position_ned(self.to_ned(self.goal))
        self.streaming = True
        self._stream_task = asyncio.create_task(self._stream_setpoints())
        await asyncio.sleep(0.5)                     # stream must run before start()
        try:
            await self.sys.offboard.start()
            self.info("offboard (GUIDED position control) active")
        except OffboardError as err:
            # ArduPilot often answers this although GUIDED accepts the setpoints.
            self.warn(f"offboard.start() answered {err._result.result}; continuing in GUIDED")

    async def _stop_stream(self):
        self.streaming = False
        if self._stream_task:
            await asyncio.gather(self._stream_task, return_exceptions=True)
            self._stream_task = None
        self.override = None

    async def _land(self, reason: str):
        self.status = "landing"
        await self._stop_stream()
        try:
            await self.sys.offboard.stop()
        except Exception as e:  # noqa: BLE001
            self.warn(f"offboard.stop(): {e!r} (continuing with land)")
        self.info(f"landing ({reason})")
        for attempt in range(3):
            try:
                await self.sys.action.land()
                break
            except Exception as e:  # noqa: BLE001
                self.warn(f"land command failed ({e!r}), retry {attempt + 1}/3")
                await asyncio.sleep(1.0)
        t_end = time.monotonic() + LAND_TIMEOUT_S
        while self.armed:
            if time.monotonic() > t_end:
                self.warn("landing timeout - drone still armed! check it in QGroundControl")
                self.status = "land-timeout"
                return
            await asyncio.sleep(0.5)
        self.t_landed = time.monotonic()
        self.status = "landed"
        self.info("landed and disarmed")

    # ------------------------------------------------------------- mission
    async def fly(self, waypoints: List[Point], takeoff_delay: float = 0.0):
        self.waypoints = waypoints
        if not waypoints:
            self.warn("no waypoints assigned - staying on the ground")
            self.status = "done"
            return
        try:
            if takeoff_delay:
                self.info(f"waiting {takeoff_delay:.0f} s before takeoff (homes are close)")
                await asyncio.sleep(takeoff_delay)
            await self._takeoff()
            await self._start_offboard()

            self.info(f"flying to start of my area ({len(waypoints)} waypoints, "
                      f"{path_length(waypoints):.0f} m)")
            self.status = "coverage"
            for i, wp in enumerate(waypoints):
                ok = await self.go_to(wp, ACCEPTANCE_RADIUS_M, f"WP {i + 1}/{len(waypoints)}")
                if ok:
                    self.reached += 1
                    if (i + 1) % 5 == 0 or i + 1 == len(waypoints):
                        self.info(f"reached WP {i + 1}/{len(waypoints)}")
                elif self.swarm.stop_event.is_set() or self.abort_reason:
                    self.missed.extend(range(i, len(waypoints)))
                    break
                else:
                    self.missed.append(i)

            if self.swarm.stop_event.is_set():
                await self._land("stop requested")
                return

            self.status = "returning"
            self.info("coverage done" if not self.abort_reason else f"aborting: {self.abort_reason}")
            assert self.home_map is not None
            home = (self.home_map[0], self.home_map[1], self.alt)
            await self.go_to(home, HOME_ACCEPTANCE_M, "home")
            await self.swarm.wait_landing_clear(self)
            await self._land("mission complete" if not self.abort_reason else self.abort_reason)
        except asyncio.CancelledError:
            await self._safe_land("cancelled")
            raise
        except Exception as e:  # noqa: BLE001
            self.abort_reason = self.abort_reason or f"error: {e!r}"
            self.warn(f"mission error: {e!r}")
            await self._safe_land("error")

    async def _safe_land(self, reason: str):
        if self.armed and self.status not in ("landing", "landed"):
            try:
                await self._land(reason)
            except Exception as e:  # noqa: BLE001
                self.warn(f"emergency land failed: {e!r}")

    async def shutdown(self):
        await self._stop_stream()
        for t in self._tasks:
            t.cancel()
        await asyncio.gather(*self._tasks, return_exceptions=True)
        self._tasks.clear()


def _mavsdk_system(grpc_port: int):
    from mavsdk import System          # imported lazily so --dry-run works without it
    return System(port=grpc_port)


# =============================================================================
# The swarm
# =============================================================================
class Swarm:
    def __init__(self, cfgs: List[dict], system_factory=None):
        if system_factory is None:
            system_factory = _mavsdk_system
        self.stop_event = asyncio.Event()
        self.drones = [Drone(c, self, system_factory) for c in cfgs]
        self.min_sep: Optional[float] = None
        self.min_sep_pair: Optional[Tuple[int, int]] = None
        self._monitor_task: Optional[asyncio.Task] = None
        self._sigint_count = 0

    # --------------------------------------------------------------- Ctrl+C
    def install_signal_handler(self):
        try:
            asyncio.get_running_loop().add_signal_handler(signal.SIGINT, self._on_sigint)
        except (NotImplementedError, RuntimeError):
            pass

    def _on_sigint(self):
        self._sigint_count += 1
        if self._sigint_count == 1:
            log("SWARM", "Ctrl+C -> stopping: every airborne drone will LAND where it is "
                         "(press Ctrl+C again to quit immediately)")
            self.stop_event.set()
        else:
            raise KeyboardInterrupt

    # ----------------------------------------------------- collision monitor
    async def collision_monitor(self):
        while True:
            self._check_pairs(time.monotonic())
            await asyncio.sleep(CONTROL_PERIOD_S)

    def _check_pairs(self, now: float):
        """One avoidance cycle.

        For every pair of airborne drones decide who yields (`lo`) and how:
          * a drone that is taking off / landing cannot be steered, so the other
            one always yields; otherwise the higher id yields;
          * HOLD  - stop in place, when the other drone will pass by;
          * EVADE - move vertically away (+ small side-step), when holding would
            not help (the other drone is heading at us) or we are already too
            close. In an emergency the priority drone also stops.
        Hysteresis keeps a decision until the drones are clearly apart."""
        live = [d for d in self.drones if d.frame_ready and d.ned is not None and d.airborne]
        demand: Dict[int, Tuple[str, "Drone"]] = {}

        def want(d: Drone, mode: str, other: Drone):
            if not d.streaming:
                return
            cur = demand.get(d.id)
            if cur is None or (mode == "evade" and cur[0] == "hold"):
                demand[d.id] = (mode, other)

        for a, b in itertools.combinations(live, 2):
            if a.streaming != b.streaming:
                hi, lo = (b, a) if a.streaming else (a, b)   # lo = the one we can steer
            else:
                hi, lo = (a, b) if a.id < b.id else (b, a)   # lower id has priority
            pa, pb = hi.map_pos(), lo.map_pos()
            dh, dv = dist2(pa, pb), abs(pa[2] - pb[2])
            d3 = math.hypot(dh, dv)
            if self.min_sep is None or d3 < self.min_sep:
                self.min_sep, self.min_sep_pair = d3, (hi.id, lo.id)
            if not lo.streaming:
                continue                     # both in takeoff/landing: nothing to steer

            engaged = bool(lo.override) and lo.override_from == hi.id
            safe = SAFE_DIST_M + (CLEAR_HYSTERESIS_M if engaged else 0.0)
            horizon = PREDICT_HORIZON_S * (2.0 if engaged else 1.0)

            rx, ry, rz = pb[0] - pa[0], pb[1] - pa[1], pb[2] - pa[2]   # hi -> lo
            va, vb = hi.map_vel(), lo.map_vel()
            vx, vy, vz = vb[0] - va[0], vb[1] - va[1], vb[2] - va[2]
            # smallest vertical gap expected within the horizon (climbing/descending drones)
            t_v = min(max(-rz / vz, 0.0), horizon) if abs(vz) > 1e-3 else 0.0
            vertical_safe = min(dv, abs(rz + vz * t_v)) >= VERTICAL_SAFE_M
            # (1) predicted horizontal closest approach with both drones moving
            vv = vx * vx + vy * vy
            tcpa = -(rx * vx + ry * vy) / vv if vv > 1e-6 else 0.0
            predicted = 0.0 < tcpa < horizon and math.hypot(rx + vx * tcpa, ry + vy * tcpa) < safe
            # (2) would hi run into lo even if lo stopped? then holding is useless
            hv = va[0] * va[0] + va[1] * va[1]
            ts = (rx * va[0] + ry * va[1]) / hv if hv > 0.04 else -1.0
            hi_hits_lo = 0.0 <= ts < horizon and \
                math.hypot(rx - va[0] * ts, ry - va[1] * ts) < safe

            if engaged:
                # Keep the decision until the other drone has PASSED. A holding /
                # evading drone is (almost) stopped, so "would we still conflict?"
                # is judged with the velocity it WOULD have if it resumed now.
                wx, wy = lo.intended_vel()
                ix, iy = wx - va[0], wy - va[1]
                iv = ix * ix + iy * iy
                ti = -(rx * ix + ry * iy) / iv if iv > 1e-6 else 0.0
                would_conflict = 0.0 < ti < horizon and \
                    math.hypot(rx + ix * ti, ry + iy * ti) < safe
                if not (dh < safe or would_conflict or hi_hits_lo):
                    continue                             # passed and clear -> resume
                mode = lo.override
                if mode == "hold":
                    if vertical_safe:
                        continue                         # altitude now separates us
                    if hi_hits_lo or dh < EMERGENCY_DIST_M:
                        mode = "evade"
                want(lo, mode, hi)
                if dh < EMERGENCY_DIST_M and dv < VERTICAL_SAFE_M:
                    want(hi, "hold", lo)
                continue

            if vertical_safe or not (dh < safe or predicted or hi_hits_lo):
                continue
            if dh < EMERGENCY_DIST_M:
                want(lo, "evade", hi)
                want(hi, "hold", lo)
            elif hi_hits_lo:
                want(lo, "evade", hi)
            else:
                want(lo, "hold", hi)

        for d in self.drones:
            if not d.streaming:
                continue
            req = demand.get(d.id)
            # dead-lock breaker: holding too long -> evade
            if req and req[0] == "hold" and d.override == "hold" \
                    and now - d.override_since > HOLD_TIMEOUT_S:
                req = ("evade", req[1])
            if req is None:
                if d.override:
                    d.info(f"clear of Drone {d.override_from} -> resuming")
                    d.override = d.override_point = d.override_from = None
                continue
            mode, other = req
            if mode == d.override and other.id == d.override_from:
                continue
            p, o = d.map_pos(), other.map_pos()
            if mode == "hold":
                d.override_point = (p[0], p[1], max(p[2], MIN_EVADE_ALT_M))
                d.holds += 1
                d.info(f"conflict with Drone {other.id} ({dist2(p, o):.1f} m) -> HOLDING")
            else:
                ux, uy = p[0] - o[0], p[1] - o[1]
                norm = math.hypot(ux, uy) or 1.0
                if not other.streaming:
                    # other drone is taking off / landing (moving vertically):
                    # get out of its column sideways, keep our altitude
                    z, push = p[2], 1.5 * SAFE_DIST_M
                else:
                    up = p[2] >= o[2]                   # move vertically AWAY from it
                    z = p[2] + EVADE_CLIMB_M if up else p[2] - EVADE_CLIMB_M
                    z = max(MIN_EVADE_ALT_M, min(z, MAX_ALTITUDE_M))
                    push = EVADE_PUSH_M if abs(z - o[2]) >= VERTICAL_SAFE_M else 2 * SAFE_DIST_M
                d.override_point = (p[0] + push * ux / norm, p[1] + push * uy / norm, z)
                d.evades += 1
                d.warn(f"Drone {other.id} {dist2(p, o):.1f} m away and closing -> "
                       f"EVADING {('up' if z > p[2] else 'down') + f' to {z:.1f} m' if abs(z - p[2]) > 0.1 else f'sideways {push:.0f} m'}")
            if not d.override:
                d.resume_speed = math.hypot(*d.map_vel()[:2])
            d.override, d.override_from, d.override_since = mode, other.id, now

    async def wait_landing_clear(self, d: Drone, timeout_s: float = 45.0):
        """Do not descend while another drone is flying below us."""
        t_end, told = time.monotonic() + timeout_s, False
        while not self.stop_event.is_set():
            p = d.map_pos()
            below = [o for o in self.drones if o is not d and o.airborne and o.frame_ready
                     and o.map_pos()[2] < p[2]
                     and dist2(o.map_pos(), p) < SAFE_DIST_M + CLEAR_HYSTERESIS_M]
            if not below:
                return
            if not told:
                d.info(f"waiting to land: Drone {below[0].id} is below me")
                told = True
            if time.monotonic() > t_end:
                d.warn("landing area still not clear after timeout - landing anyway")
                return
            await asyncio.sleep(0.5)

    # ------------------------------------------------------------ main flow
    async def run(self, split_mode: str, dry_run: bool, layers: bool, allow_partial: bool):
        self.install_signal_handler()

        if dry_run:
            homes = [(0.0, 0.0)] * len(self.drones)
            segments, how = load_plan(WAYPOINT_DIR, len(self.drones), split_mode)
            plan = self._finalise_plan(assign_segments(segments, homes), layers)
            self._print_plan(plan, how)
            write_flown_waypoints(self.drones, plan)
            plot_mission(self.drones, plan, actual=False)
            log("SWARM", "dry run finished - no drone was contacted")
            return

        # 1. connect
        for d in self.drones:                         # one mavsdk_server per drone
            await d.init_system()
            await asyncio.sleep(0.3)
        log("SWARM", "=== connecting to drones ===")
        ok = await asyncio.gather(*[d.connect() for d in self.drones])
        for d, good in zip(self.drones, ok):
            log("SWARM", f"{d.tag} ({d.url}): {'OK' if good else 'FAILED'}")
        if not all(ok):
            if not allow_partial or not any(ok):
                log("SWARM", "aborting: not every drone is ready (use --allow-partial to "
                             "fly with the ones that are)")
                return
            self.drones = [d for d, good in zip(self.drones, ok) if good]

        # 2. telemetry + shared frame
        for d in self.drones:
            d.start_telemetry()
            await d.set_rates()
        try:
            await asyncio.gather(*[d.measure_origin() for d in self.drones])
            lat0, lon0 = MAP_ORIGIN_LATLON or self.drones[0].origin_latlon
            for d in self.drones:
                d.set_map_origin(lat0, lon0)
            self._print_frames(lat0, lon0)

            # 3. plan
            homes = [(d.home_map[0], d.home_map[1]) for d in self.drones]
            segments, how = load_plan(WAYPOINT_DIR, len(self.drones), split_mode)
            plan = self._finalise_plan(assign_segments(segments, homes), layers)
            self._print_plan(plan, how)
            write_flown_waypoints(self.drones, plan)

            delays = self._takeoff_delays(homes)

            # 4. fly
            for d in self.drones:
                d.status = "ready"
            log("SWARM", "=== all drones ready: taking off ===")
            self._monitor_task = asyncio.create_task(self.collision_monitor())
            await asyncio.gather(*[d.fly(wp, delays[i])
                                   for i, (d, wp) in enumerate(zip(self.drones, plan))])
        finally:
            if self._monitor_task:
                self._monitor_task.cancel()
                await asyncio.gather(self._monitor_task, return_exceptions=True)
            for d in self.drones:
                await d._safe_land("shutdown")
                if d.status in ("landed", "ready", "init"):
                    d.status = "done"
            self._print_summary()
            for d in self.drones:
                await d.shutdown()
        plot_mission(self.drones, [d.waypoints for d in self.drones], actual=True)

    # ------------------------------------------------------------- helpers
    def _finalise_plan(self, segments: List[List[Point]], layers: bool) -> List[List[Point]]:
        plan = []
        for d, seg in zip(self.drones, segments):
            if layers:
                plan.append([(x, y, d.alt) for x, y, _ in seg])
            else:
                if seg:
                    d.alt = min(seg[0][2], MAX_ALTITUDE_M)
                plan.append([(x, y, min(z, MAX_ALTITUDE_M)) for x, y, z in seg])
        if not layers:
            log("SWARM", "WARNING: altitude layers disabled - all drones share one altitude, "
                         "collision avoidance is the only protection")
        return plan

    def _takeoff_delays(self, homes) -> List[float]:
        close = any(dist2(a, b) < SAFE_DIST_M for a, b in itertools.combinations(homes, 2))
        if close:
            log("SWARM", f"WARNING: some drones start closer than {SAFE_DIST_M} m - "
                         f"staggering takeoffs by {STAGGER_TAKEOFF_S:.0f} s")
            return [i * STAGGER_TAKEOFF_S for i in range(len(homes))]
        return [0.0] * len(homes)

    def _print_frames(self, lat0, lon0):
        log("SWARM", f"map origin (0,0) = {lat0:.7f}, {lon0:.7f}")
        for d in self.drones:
            h = d.home_map
            log("SWARM", f"{d.tag}: local origin offset E={d.off_e:+.2f} m N={d.off_n:+.2f} m, "
                         f"home in map = ({h[0]:+.1f}, {h[1]:+.1f})")

    def _print_plan(self, plan, how):
        log("SWARM", f"plan: {how}")
        for d, wp in zip(self.drones, plan):
            if not wp:
                log("SWARM", f"{d.tag}: NO waypoints")
                continue
            log("SWARM", f"{d.tag}: {len(wp):3d} waypoints, {path_length(wp):6.1f} m, "
                         f"altitude {wp[0][2]:.1f} m, start ({wp[0][0]:+.1f}, {wp[0][1]:+.1f})")

    def _print_summary(self):
        lines = ["", "=" * 72, "MISSION SUMMARY", "=" * 72]
        for d in self.drones:
            t = (d.t_landed - d.t_takeoff) if d.t_takeoff and d.t_landed else None
            parts = [f"{d.tag}: status={d.status}" + (f" ({d.abort_reason})" if d.abort_reason else ""),
                     f"WPs {d.reached}/{len(d.waypoints)} reached, {len(d.missed)} missed",
                     f"planned {path_length(d.waypoints):.0f} m, flown {d.flown_m:.0f} m",
                     f"flight time {t:.0f} s" if t is not None else "flight time -",
                     f"holds {d.holds}, evades {d.evades}, held {d.hold_time:.0f} s"]
            lines.append(" | ".join(parts))
        if self.min_sep is not None:
            lines.append(f"Minimum 3-D separation between airborne drones: {self.min_sep:.1f} m "
                         f"(Drones {self.min_sep_pair[0]} & {self.min_sep_pair[1]})")
        lines.append("=" * 72)
        text = "\n".join(lines)
        print(text, flush=True)
        try:
            with open(os.path.join(WAYPOINT_DIR, "mission_summary.txt"), "w") as f:
                f.write(text.strip() + "\n")
        except OSError as e:
            log("SWARM", f"could not write mission_summary.txt: {e}")


# =============================================================================
# Output files
# =============================================================================
def write_flown_waypoints(drones: List[Drone], plan: List[List[Point]]):
    os.makedirs(WAYPOINT_DIR, exist_ok=True)
    for d, wp in zip(drones, plan):
        path = os.path.join(WAYPOINT_DIR, f"flown_waypoints_drone{d.id}.csv")
        with open(path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["seq", "x_local_m", "y_local_m", "z_m"])
            for i, (x, y, z) in enumerate(wp):
                w.writerow([i, f"{x:.3f}", f"{y:.3f}", f"{z:.1f}"])


def plot_mission(drones: List[Drone], plan: List[List[Point]], actual: bool):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        log("SWARM", "matplotlib not installed - skipping plot")
        return
    colors = ["tab:blue", "tab:orange", "tab:green", "tab:red", "tab:purple"]
    fig, ax = plt.subplots(figsize=(9, 8))
    for k, (d, wp) in enumerate(zip(drones, plan)):
        c = colors[k % len(colors)]
        if wp:
            ax.plot([p[0] for p in wp], [p[1] for p in wp], "-o", ms=2.5, lw=1.2, color=c,
                    alpha=0.45, label=f"Drone {d.id} planned ({wp[0][2]:.0f} m)")
            ax.plot(wp[0][0], wp[0][1], "^", color=c, ms=9)
        if actual:
            xs, ys = read_map_track(d.id)
            if xs:
                ax.plot(xs, ys, "-", lw=1.4, color=c, label=f"Drone {d.id} actual")
    ax.set_aspect("equal")
    ax.grid(alpha=0.3)
    ax.set_xlabel("East / map x (m)")
    ax.set_ylabel("North / map y (m)")
    ax.set_title("Swarm coverage - planned vs actual" if actual else "Swarm coverage plan (dry run)")
    ax.legend(fontsize=8, loc="best")
    out = os.path.join(WAYPOINT_DIR,
                       "swarm_planned_vs_actual.png" if actual else "swarm_plan_dry_run.png")
    fig.savefig(out, dpi=150, bbox_inches="tight")
    plt.close(fig)
    log("SWARM", f"plot saved -> {out}")


def read_map_track(drone_id: int) -> Tuple[List[float], List[float]]:
    path = os.path.join(WAYPOINT_DIR, f"actual_path_drone{drone_id}.csv")
    xs, ys = [], []
    if os.path.exists(path):
        with open(path, newline="") as f:
            for row in csv.DictReader(f):
                if row.get("map_e"):
                    xs.append(float(row["map_e"]))
                    ys.append(float(row["map_n"]))
    return xs, ys


# =============================================================================
# Entry point
# =============================================================================
def parse_args():
    ap = argparse.ArgumentParser(description="Multi-drone BCD coverage mission (MAVSDK, ArduPilot)")
    ap.add_argument("--dry-run", action="store_true", help="plan and plot only, no drones")
    ap.add_argument("--plot-only", action="store_true", help="re-plot the last mission logs")
    ap.add_argument("--split", choices=["auto", "length", "planner"], default="auto",
                    help="auto: keep planner split if balanced, else re-split by length (default)")
    ap.add_argument("--no-layers", action="store_true",
                    help="fly every drone at the CSV altitude instead of its own layer")
    ap.add_argument("--num-drones", type=int, default=len(DRONES),
                    help="use only the first N drones from DRONES")
    ap.add_argument("--allow-partial", action="store_true",
                    help="fly with the drones that connected even if some failed")
    ap.add_argument("--waypoint-dir", default=None, help=f"default {WAYPOINT_DIR}")
    return ap.parse_args()


async def amain(args):
    cfgs = DRONES[:max(1, min(args.num_drones, len(DRONES)))]
    swarm = Swarm(cfgs)
    if args.plot_only:
        segs = [read_waypoints(os.path.join(WAYPOINT_DIR, f"flown_waypoints_drone{c['id']}.csv"))
                or [] for c in cfgs]
        plot_mission(swarm.drones, segs, actual=True)
        return
    await swarm.run(args.split, args.dry_run, not args.no_layers, args.allow_partial)


def main():
    global WAYPOINT_DIR
    args = parse_args()
    if args.waypoint_dir:
        WAYPOINT_DIR = os.path.expanduser(args.waypoint_dir)
    try:
        asyncio.run(amain(args))
    except KeyboardInterrupt:
        print("\n[WARN] forced quit - check in QGroundControl that every drone has landed!")


if __name__ == "__main__":
    main()
