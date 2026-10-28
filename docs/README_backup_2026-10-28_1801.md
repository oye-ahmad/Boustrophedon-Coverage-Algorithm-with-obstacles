# 🚁 Boustrophedon Coverage with Obstacles: Single & Multi-UAV

> Part of the FYP **"AI-Powered Cooperative Drone Swarm for Autonomous Area Coverage and Object Detection"** (UET Taxila, Computer Engineering).
> Planner: ROS 2 (Humble) BCD coverage nodes · Flight: ArduPilot SITL + Gazebo · Control: MAVSDK-Python (GUIDED offboard)

**Status (28 Oct 2026):** a **3-drone swarm** covers an obstacle map in SITL. The work is split evenly and each drone flies at its own altitude, with 3-D collision avoidance, a safe return home, and a full post-flight metrics report. Latest result: **98.5 % of the target area covered**, with path-tracking RMSE **0.12–0.18 m**.

---

## Table of contents
1. [Repository structure](#1-repository-structure)
2. [Progress log](#2-progress-log)
3. [How to run the 3-drone mission (step by step)](#3-how-to-run-the-3-drone-mission-step-by-step)
4. [How the swarm mission works](#4-how-the-swarm-mission-works)
5. [Latest mission results (28 Oct 2026)](#5-latest-mission-results-28-oct-2026)
6. [Problems we faced and how we solved them](#6-problems-we-faced-and-how-we-solved-them)
7. [Known issues and next steps](#7-known-issues-and-next-steps)
8. [Earlier results](#8-earlier-results)
9. [Single-UAV BCD coverage (detailed write-up)](#single-uav-autonomous-coverage-using-boustrophedon-cellular-decomposition)

---

## 1. Repository structure

```text
coverage_ws/
├── src/
│   ├── coverage_planner/                     # ROS 2 package (ament_python)
│   │   └── coverage_planner/
│   │       ├── bcd_coverage_node.py          # grid-based BCD, single UAV  -> waypoints.csv
│   │       ├── bcd_geometric_planner.py      # geometric (polygon) BCD
│   │       ├── coverage_planner_node.py      # first simple lawnmower planner
│   │       ├── bcd_coverage_multi_node.py    # BCD + split into 3 regions -> waypoints_drone{1,2,3}.csv
│   │       └── multi_drone_formation_controller.py   # early MAVROS formation test (arming issue, not used)
│   │
│   ├── swarm_mission_mavsdk.py               # ★ CURRENT 3-drone mission (MAVSDK, ArduPilot GUIDED)
│   ├── multi_drone_offboard_mission.py       # previous 3-drone script (kept for reference)
│   ├── region_partitioner.py                 # length-balanced split helper (older)
│   ├── plot_combined_coverage.py             # older 3-drone plotter
│   ├── mission_executor.py                   # single-UAV MAVSDK mission (AUTO mode) + GPS logging
│   ├── mavsdk_waypoint_follower.py           # single-UAV offboard waypoint follower
│   └── path_plotter.py                       # single-UAV planned-vs-actual plot
│
└── docs/
    ├── images/                               # figures used in this README
    └── results/2026-10-28/                   # metrics, summary and log of the latest mission
```

ROS 2 entry points (`ros2 run coverage_planner <name>`): `bcd_coverage_node`, `bcd_geometric_node`, `coverage_planner_node`, `multi_drone_node`, `swarming_node`.

All mission data is read from and written to **`~/coverage_map/`**: the map, the waypoint CSVs, the flight logs, the plots and the metrics.

---

## 2. Progress log

| Date | Milestone |
|---|---|
| 12 Aug 2026 | First lawnmower planner on a ROS 2 occupancy map |
| 13–15 Aug | Grid-based **BCD with obstacles** and **geometric BCD**; single-UAV SITL flight with GPS logging and planned-vs-actual plot |
| 20–21 Aug | First swarm attempt: MAVROS formation controller (stopped: arming problem never solved) |
| 22–24 Aug | **3-drone BCD coverage with MAVSDK**: 3-way split, separate gRPC ports, hold-based collision avoidance, first combined plot |
| 28 Oct | **`swarm_mission_mavsdk.py`**: code review of the swarm stack, 10+ bugs fixed (see [section 6](#6-problems-we-faced-and-how-we-solved-them)), equal-length work split, altitude layers, predictive 3-D collision avoidance, safe return, failsafes, full metrics report; successful 3-drone SITL mission on the obstacle map |

---

## 3. How to run the 3-drone mission (step by step)

You need **5 terminals**. In every terminal that uses ROS 2, run:
```bash
source /opt/ros/humble/setup.bash
source ~/coverage_ws/install/setup.bash     # otherwise: "Package 'coverage_planner' not found"
```
(Tip: put both lines at the end of `~/.bashrc`.)

**One-time checks**
```bash
pip3 show mavsdk | grep Version     # tested with mavsdk-python 3.15.3 (gRPC API)
pip3 install matplotlib             # plots + coverage map
```

- The Gazebo world must contain 3 drones, ideally **≥ 5 m apart**. Closer than 3 m and the script takes off one drone at a time, 8 s apart. Each drone's ArduPilot plugin needs its own `fdm_port_in`: 9002, 9012 and 9022.
- **Close QGroundControl.** It grabs UDP 14550, which drone 1 needs (see the tip at the end of this section).

**Step 1: Gazebo (Terminal 1)**
```bash
gz sim -v4 -r <your_3_drone_world>.sdf
```

**Step 2: three ArduPilot SITL instances (Terminals 2, 3, 4)**
```bash
cd ~/ardupilot/ArduCopter
sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON -I0 --sysid 1 --console   # Drone 1 -> UDP 14550
sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON -I1 --sysid 2 --console   # Drone 2 -> UDP 14560
sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON -I2 --sysid 3 --console   # Drone 3 -> UDP 14570
```
> ⚠ **Do not add `--out=udp:...:14550/14560/14570`.** `-I N` already sends to port 14550 + 10·N. A second `--out` duplicates every packet, and that crashed `mavsdk_server` (see [problem 12](#6-problems-we-faced-and-how-we-solved-them)).
> Check by typing `output` in each MAVProxy console: each port should appear only once.

Wait until every console reports a GPS fix / `EKF3 IMU0 is using GPS` (about 30–60 s).

**Step 3: waypoints (only if you want a new plan)**
```bash
python3 ~/coverage_map/generate_map_with_obstacles.py         # optional: new obstacle map
ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=$HOME/coverage_map/coverage_area.yaml   # leave running
ros2 run nav2_util lifecycle_bringup map_server               # activates the map (prints nothing)
ros2 run coverage_planner multi_drone_node                    # writes ~/coverage_map/waypoints_drone{1,2,3}.csv, then Ctrl+C
```
If those CSVs (or a single `waypoints.csv`) already exist, skip this step. The mission script re-balances the work anyway.

**Step 4: dry run (nothing flies)**
```bash
cd ~/coverage_ws/src
python3 swarm_mission_mavsdk.py --dry-run          # -> ~/coverage_map/swarm_plan_dry_run.png
```

**Step 5: fly**
```bash
pkill -f mavsdk_server                             # clear leftovers from earlier runs
python3 swarm_mission_mavsdk.py 2>&1 | grep --line-buffered -v "not-existing" | tee ~/coverage_map/run.log
```

- The `grep` hides the harmless MAVSDK message `Received ack for not-existing command`.
- **Ctrl+C once** makes every airborne drone land where it is. Press it twice only in an emergency.
- Useful options: `--num-drones 2`, `--allow-partial` (fly with whichever drones connected), `--split planner|length|auto`, `--no-layers`, `--plot-only` (re-plot and recompute the metrics from the last logs).

**Step 6: results in `~/coverage_map/`**

| File | Content |
|---|---|
| `mission_metrics.txt` / `.csv` | all post-flight metrics (section 5) |
| `coverage_map.png` | area covered per drone, overlap, missed cells |
| `swarm_planned_vs_actual.png` | planned vs flown path of every drone |
| `mission_summary.txt` | waypoints reached/missed, holds/evades, minimum separation |
| `actual_path_droneN.csv` | raw log: GPS, map-frame position, status, battery |
| `flown_waypoints_droneN.csv` | the waypoints each drone actually received |

**Tip (QGroundControl while flying):** add a second output to each SITL with a *different* port (e.g. MAVProxy `output add 127.0.0.1:14551` / `14561` / `14571`). In QGC, add UDP links for those ports and turn off auto-connect to 14550.

---

## 4. How the swarm mission works

```text
 /map (OccupancyGrid)
        │  bcd_coverage_multi_node  (ROS 2)
        ▼
 BCD cells → lawnmower paths → greedy cell order → waypoints_drone{1,2,3}.csv
        │
        │  swarm_mission_mavsdk.py  (one mavsdk_server per drone, gRPC 50051/52/53)
        ▼
 connect + health check (all drones)  →  measure each drone's local origin → shared map frame
        ▼
 join all CSVs into one path → cut into N pieces of EQUAL LENGTH → give each piece to the nearest drone
        ▼
 arm → take off to 3 m → climb to own layer (8 / 11 / 14 m) → fly piece (10 Hz position setpoints)
        │         ▲
        │         └── collision monitor (10 Hz, 3-D, predictive): HOLD / EVADE / resume
        ▼
 fly home at own layer → wait until nobody is below → land → metrics + plots
```

**4.1 Shared map frame.** Every ArduPilot instance measures position from *its own* local origin, where it booted. At start-up the script computes each origin in lat/lon (`current GPS − current NED offset`) and converts it into one shared map frame. Map point (x, y) is then the same spot on the ground for every drone. Map (0, 0) = drone 1's origin, unless `MAP_ORIGIN_LATLON` is set. Map x = **East**, y = **North** (ROS ENU).

**4.2 Equal work split.** The planner's paths are joined in their original order and cut into equal-length pieces, cutting inside a row if needed. Each piece is assigned (and possibly reversed) so that the longest home-to-start trip is as short as possible, and two drones never start at the same point. Waypoint counts can differ: a waypoint is a corner, so a long straight row needs only 2. In the latest run drone 2 had 37 waypoints and drone 1 had 14, but **each drone flew 237.1 m**.

**4.3 Altitude layers.** Each drone takes off to 3 m, then climbs straight up to its own cruise altitude (8 / 11 / 14 m, set in `DRONES`). This removes most conflicts by design. Turn it off with `--no-layers`.

**4.4 Collision avoidance (10 Hz, 3-D).** For every pair of airborne drones:

- **Who gives way:** a drone that is taking off or landing cannot be steered, so the other one always gives way. Otherwise the drone with the higher id gives way.
- **Conflict test:** a conflict is predicted from the velocities over a 3 s horizon, both horizontally and vertically, not only from the current distance.
- **HOLD:** if the other drone will pass by, the drone that gives way stops in place.
- **EVADE:** if the other drone is heading straight at it, or it is already within 2.5 m, it moves 4 m vertically away. It moves sideways instead if the other drone is taking off or landing. In an emergency the priority drone stops too.
- **No flicker:** a decision is kept until the other drone has really passed. This is judged with the speed the drone *would* have if it resumed, so there is no hold/resume flicker.
- **No dead-lock:** a hold longer than 20 s turns into an evade.

**4.5 Return and landing.** Drones fly home at their own layer altitude, wait until no other drone is below them, then land. We don't use ArduPilot RTL because it would send all drones to the same RTL altitude.

**4.6 Safety and robustness.**

- **Timeouts:** connect, health check, climb, every waypoint and landing all have timeouts. Time spent holding for another drone doesn't count against the waypoint timeout.
- **Arming:** up to 3 tries.
- **Battery failsafe:** each drone returns home if its battery drops below 25 %.
- **Ctrl+C:** makes every drone land.
- **Dead connection:** if a drone's `mavsdk_server` dies or its telemetry freezes for more than 5 s, that drone is taken out of the mission with one clear message, and the others continue.
- **Streams:** setpoints are sent by a dedicated 10 Hz task, and telemetry streams restart automatically.

**4.7 Metrics.** These are computed from the flight logs, so `--plot-only` can redo them.

- **Per drone:** flight and coverage time, distance flown vs planned, average speed, area covered (swath model), path-tracking error (mean / RMSE / max), and battery start → end, % used, drain %/min, voltage, mAh and estimated time left.
- **Swarm:** total mission time, coverage makespan, target area, total area covered (union), coverage %, overlap, total distance, total battery, area per % battery, and minimum separation.

**Main settings** (top of `swarm_mission_mavsdk.py`)

| Setting | Default | Meaning |
|---|---|---|
| `DRONES` | 3 drones, UDP 14550/60/70, gRPC 50051/52/53, 8/11/14 m | connection + layer altitude per drone |
| `TAKEOFF_ALTITUDE_M` | 3.0 | ArduPilot takeoff height before climbing to the layer |
| `ACCEPTANCE_RADIUS_M` | 1.0 | waypoint reached (3-D) |
| `SAFE_DIST_M` / `EMERGENCY_DIST_M` | 5.0 / 2.5 | horizontal separation wanted / emergency |
| `VERTICAL_SAFE_M` | 2.0 | vertical gap that counts as separated |
| `SWATH_WIDTH_M` | 3.0 | camera ground width for area metrics (= row spacing). Real camera: 2·h·tan(HFOV/2) |
| `BATTERY_FAILSAFE_PCT` | 25 | return-home threshold |
| `MAP_ORIGIN_LATLON` | None | lat/lon of map (0, 0); None = drone 1's origin |

---

## 5. Latest mission results (28 Oct 2026)

Setup: obstacle map 40 × 40 m at 0.2 m/px (`generate_map_with_obstacles.py`), 3 drones spawned about 5 m apart, ArduPilot SITL + Gazebo, mavsdk-python 3.15.3, row spacing 3 m. The planner's 3 CSVs were unbalanced (drone 2 had only 8 waypoints), so the script re-split the full 711 m path into **3 × 237.1 m**.

### 5.1 Plan from the dry run
<img src="docs/images/2026-10-28_swarm_plan_dry_run.png" alt="Dry-run plan" width="600"/>

*The dry run doesn't know the real home positions yet, so the piece-to-drone assignment (colours) can change at flight time. The three pieces themselves are the same.*

### 5.2 Planned vs actual flight paths
<img src="docs/images/2026-10-28_swarm_planned_vs_actual.png" alt="Planned vs actual" width="600"/>

*Light lines are planned, dark lines are flown; ▲ marks each drone's start. Straight diagonals are trips to and from home, and the planner's jumps between cells.*

### 5.3 Area coverage map
<img src="docs/images/2026-10-28_coverage_map.png" alt="Coverage map" width="600"/>

### 5.4 Mission summary

| | Drone 1 | Drone 2 | Drone 3 | Swarm |
|---|---|---|---|---|
| Layer altitude | 8 m | 11 m | 14 m | |
| Waypoints reached | 14 / 14 | 35 / 37 (2 timed out) | 17 / 17 | 66 / 68 |
| Planned / flown distance | 237 / 277 m | 237 / 274 m | 237 / 263 m | 814 m flown |
| Flight time (takeoff → landing) | 7:09 | 13:20 | 8:50 | **13:20** total mission |
| Coverage time | 4:29 | 7:35 | 4:54 | 7:56 makespan |
| Average coverage speed | 0.88 m/s | 0.50 m/s | 0.80 m/s | |
| Area covered | 749 m² | 702 m² | 679 m² | **1752 m²** (union) |
| Path-tracking error (RMSE / max) | 0.18 / 0.60 m | 0.12 / 0.90 m | 0.15 / 0.73 m | |
| Holds / evades | 0 / 0 | 2 / 0 (held 16 s) | 0 / 0 | min separation **3.0 m** |
| End status | landed | land timeout* | land timeout* | |

- **Coverage of target area: 98.5 %** (target 1744 m², the planned paths × 3 m swath)
- **Overlap: 346 m² (19.7 %)**, mainly from the planner's long jumps between cells being flown as coverage (see section 7)
- **Battery: 0 % used**, because the SITL battery wasn't being simulated in this run, so no drain figures
- *Land timeout: the script stopped waiting after 120 s; see section 7.

Full files: [`docs/results/2026-10-28/`](docs/results/2026-10-28/): `mission_metrics.txt`, `mission_metrics.csv`, `mission_summary.txt`, `run_log.txt`.

**Observations**

- Path tracking is excellent (RMSE ≤ 0.18 m), and all assigned area except 2 corner waypoints was covered.
- Equal *distance* is not equal *time*. ArduPilot slows down at every waypoint, so drone 2, with many short legs (one corner every ~6.5 m versus every ~18 m for drone 1), took 7:35 of coverage versus 4:29.
- Takeoff to 3 m took about 45 s and speeds were 0.5–0.9 m/s. That strongly suggests Gazebo was running **slower than real time** with 3 drones; check the real-time factor (RTF) in the Gazebo window.

---

## 6. Problems we faced and how we solved them

### 6.1 Bugs found in the previous swarm code (code review, 28 Oct)

| # | Problem | Effect | Fix in `swarm_mission_mavsdk.py` |
|---|---|---|---|
| 1 | **X/Y axes swapped**: `north = x_local, east = y_local`, but the map is ENU (x = East) | every path mirrored across the diagonal; invisible on a square empty map, dangerous with obstacles | `north = y`, `east = x` |
| 2 | **Each drone uses its own local origin** for NED setpoints | if drones spawn in different places, the same map point is a different ground point for each drone | measure every origin at start-up and convert to one shared map frame |
| 3 | **Work split by cell count**, not by work (`bcd_coverage_multi_node.partition_cells`) | unequal workloads; with fewer cells than drones (e.g. an empty map gives 1 cell) drones 2 and 3 got **no waypoints** | re-split the whole path by **equal length**; nearest-drone assignment |
| 4 | **All drones at the same altitude**, and avoidance was only "higher id waits" | the lower id never gave way, so they could still collide; threshold 4 m in code vs 5 m in docs | altitude layers + predictive 3-D hold/evade avoidance |
| 5 | Errors silently ignored (`except: pass`), no timeouts (the climb loop could hang forever) | script hangs or fails silently | timeouts everywhere, clear one-line warnings |
| 6 | All drones would land via the same RTL altitude, and no landing-order check | collision risk on return | return at own layer, wait until no drone is below, then land |

### 6.2 Problems found while testing the new logic (simulated drones)

| # | Problem | Fix |
|---|---|---|
| 7 | Head-on at the same altitude: the waiting drone sat in the priority drone's path, **1.0 m** minimum separation | if the other drone is heading straight at you, EVADE (move vertically) early instead of holding; result **6.6 m** |
| 8 | Drone landing through another drone's layer: **2.5 m** separation | drones taking off or landing get right of way; vertical closing is predicted; drones evade sideways from a descending drone; "wait until nobody is below" before landing |
| 9 | Hold / resume **flicker** every 0.2 s | keep the decision until the other drone has passed, judged with the drone's *intended* velocity and its speed before it yielded |
| 10 | Transit to the first waypoint counted as coverage (false 7–8 m "tracking error") | separate `transit` status, excluded from the metrics |

### 6.3 Problems during SITL bring-up

| # | Symptom | Cause | Fix |
|---|---|---|---|
| 11 | `Package 'coverage_planner' not found` | workspace not sourced in that terminal | `source ~/coverage_ws/install/setup.bash` (each terminal, or in `~/.bashrc`) |
| 12 | `mavsdk_server` **crashed** at takeoff; endless `Connection refused`; every MAVSDK ack printed 3× | extra `--out=udp:...:145x0` on `sim_vehicle.py` duplicated the default output, so every packet arrived twice | removed `--out` (`-I N` already uses 14550 + 10·N); script no longer calls `set_maximum_speed` or the telemetry-rate requests (off by default); separate MAVSDK component id per drone |
| 13 | Drones stopped climbing at 5.5–5.9 m → `climb timeout` | ArduPilot takeoff altitude not applied, and/or telemetry frozen | take off to **3 m** (like the old working script), then climb to the layer with position setpoints; climb-stall tolerance |
| 14 | One dead drone flooded the log | telemetry retry loop | dead server detected after 3 failures → one message, the drone is removed, others continue |
| 15 | Hundreds of `Received ack for not-existing command` lines | harmless: MAVProxy forwards acks for its own requests | filter with `grep -v "not-existing"` |

---

## 7. Known issues and next steps

| Issue (from the 28 Oct run) | Plan |
|---|---|
| Drones 2 and 3 reported **"landing timeout – still armed"** after 120 s, and drone 2 timed out flying home | probably Gazebo running slower than real time (all timeouts use wall-clock time). Check the RTF; make timeouts scale with measured speed; check ArduPilot `LAND_SPEED` / disarm settings |
| **Overlap 19.7 %**: the planner's long straight jumps between cells (e.g. (2, −19.5) → (6, 19)) are flown and logged as coverage, and adjacent pieces share a boundary column | mark transitions between cells as transit; better cell ordering (TSP / GTSP instead of greedy) |
| Balanced by distance, not time (drone 2 finished last) | balance by **estimated time** = distance / speed + a cost per corner |
| Battery not simulated (stayed at 100 %) | enable SITL battery simulation (`SIM_BATT_*`, `BATT_CAPACITY`) to get real drain figures |
| Straight transitions between cells are not checked for collisions | check transitions against the inflated map, plan around obstacles |
| Control is centralised (one script) | **next: ROS 2 architecture**: one node per drone, shared state over DDS, distributed task allocation (auction / CBBA or DARP), reassigning a failed drone's area |
| Detection not yet connected to flight | camera in Gazebo → YOLO node → geotagged detections |

---

## 8. Earlier results

**BCD Geometric-Based**

<img width="2537" height="2562" alt="coverage_plan" src="https://github.com/user-attachments/assets/19173e6f-b68f-4ca7-a6b3-bbdf6244ef9c" />

**BCD Grid-Based**

<img width="1279" height="1048" alt="planned_vs_actual" src="https://github.com/user-attachments/assets/66e87b5b-8bc5-4311-9d44-d92b49d56629" />

**Multi-UAV BCD coverage (24 Aug 2026, previous script)**

<img width="1279" height="1048" alt="combined_planned_vs_actual" src="https://github.com/user-attachments/assets/6412641e-cb6e-4108-9710-bbf6a5eecbf6" />

---

# Single-UAV Autonomous Coverage Using Boustrophedon Cellular Decomposition

## 1. Abstract

Coverage Path Planning (CPP) is the problem of generating a trajectory that enables an autonomous vehicle to systematically visit an entire area of interest while avoiding obstacles. It is fundamentally different from conventional point-to-point navigation because the objective is not simply to reach a destination, but to maximize useful area coverage while minimizing unnecessary motion and maintaining safe obstacle clearance.

This work presents the design and implementation of a single-UAV autonomous coverage system based on **Boustrophedon Cellular Decomposition (BCD)**. The system integrates map-based coverage planning with autonomous UAV mission execution in a simulated Gazebo environment. The planner divides the accessible workspace into simpler cells and generates a back-and-forth, lawnmower-style trajectory within each cell. The resulting waypoints are then passed to the UAV mission layer for autonomous execution.

The implementation is organized as a complete planning-to-execution pipeline: map processing, obstacle handling, BCD decomposition, coverage-path generation, waypoint export, coordinate conversion, autonomous mission execution, GPS logging, and planned-versus-actual trajectory visualization. The project repository contains the coverage-planning and UAV execution components required for this workflow.

The final objective of the work is to demonstrate a fully autonomous single-UAV coverage mission in Gazebo and quantitatively/visually validate the executed trajectory against the generated coverage plan. The architecture is deliberately developed with future multi-UAV swarm coverage in mind, and several limitations of the current single-agent implementation are identified as considerations for the next development stage.

---

# 2. Introduction

Autonomous aerial vehicles are increasingly used for applications such as agricultural monitoring, infrastructure inspection, surveillance, environmental monitoring, and search-and-rescue operations. Many of these applications require an UAV to systematically cover a two-dimensional region rather than simply navigate between a small number of predefined destinations.

A suitable coverage planner must therefore answer three questions:

1. **Which portions of the environment are safe to cover?**
2. **How should the accessible area be divided into manageable regions?**
3. **What trajectory should the UAV follow to efficiently cover those regions?**

Boustrophedon Cellular Decomposition provides a natural solution to this problem. The environment is decomposed into simpler cells according to changes in free-space connectivity, after which a back-and-forth sweep is performed inside each cell. The resulting trajectory resembles the motion of a farmer moving a plough across a field.

The repository developed for this work is specifically focused on Boustrophedon coverage with obstacles and contains both geometric and grid-based BCD implementations.

For the autonomous UAV implementation, the grid-based approach is particularly useful because it can operate directly on a ROS2 occupancy map and generate coverage waypoints that can subsequently be executed by the simulated UAV.

---

# 3. Project Objectives

The primary objectives of this work are:

* Implement a Boustrophedon Cellular Decomposition coverage planner.
* Support environments containing obstacles.
* Generate an ordered lawnmower-style coverage path.
* Integrate the planner with a simulated UAV.
* Execute the generated coverage mission autonomously in Gazebo.
* Maintain a constant coverage altitude.
* Convert local coverage coordinates into global geographic coordinates when required.
* Log the UAV's actual GPS trajectory during flight.
* Compare the planned and actual trajectories.
* Identify limitations of the single-UAV implementation.
* Establish design considerations for future multi-UAV swarm coverage.

The intended final deliverable is therefore not only a path-planning algorithm, but a complete autonomous coverage pipeline.

---

# 4. System Architecture

The overall system can be represented as:

```text
                 Environment / Gazebo
                         │
                         ▼
                  Occupancy Map
                         │
                         ▼
                    ROS2 /map
                         │
                         ▼
               BCD Coverage Planner
                         │
              ┌──────────┴──────────┐
              │                     │
              ▼                     ▼
       Cell Decomposition     Coverage Path
              │                     │
              └──────────┬──────────┘
                         ▼
                  waypoints.csv
                         │
                         ▼
                 Coordinate Conversion
                         │
                         ▼
                    MAVSDK Mission
                         │
                         ▼
                    ArduPilot SITL
                         │
                         ▼
                       UAV
                         │
                         ▼
                  GPS Telemetry Log
                         │
                         ▼
                actual_path.csv
                         │
                         ▼
             Planned vs Actual Plot
```

The repository currently separates these responsibilities across the planner, mission executor, waypoint follower, and path plotting components.

This modular structure is beneficial because the coverage planner can be improved independently from the UAV execution layer.

---

# 5. Boustrophedon Cellular Decomposition

## 5.1 Basic Principle

BCD divides a complex free-space region into simpler cells that can individually be covered using parallel sweeps.

A conceptual representation is:

```text
┌───────────────────────────────────────┐
│                                       │
│      ┌──────────────┐                 │
│      │   Obstacle   │                 │
│      │              │                 │
│      └──────────────┘                 │
│                                       │
│                                       │
└───────────────────────────────────────┘
```

The presence of obstacles can cause changes in the connectivity of the free space. These changes define decomposition events and result in separate coverage cells.

Once decomposition is complete, each cell can be covered using a lawnmower trajectory:

```text
→ → → → → → → →
                ↓
← ← ← ← ← ← ← ←
↓
→ → → → → → → →
                ↓
← ← ← ← ← ← ← ←
```

This approach is widely used for systematic coverage because it naturally produces paths that sweep across the complete accessible region rather than repeatedly navigating between isolated target points.

---

# 6. Grid-Based BCD Implementation

The practical ROS2 implementation receives an occupancy grid through:

```text
/map
```

The map is represented using `nav_msgs/OccupancyGrid`. The planner converts the ROS occupancy map into a two-dimensional grid and classifies cells as free or occupied.

The implemented pipeline is:

```text
OccupancyGrid
      ↓
Python grid representation
      ↓
Obstacle inflation
      ↓
Column-wise BCD decomposition
      ↓
Cell generation
      ↓
Lawnmower path generation
      ↓
Greedy cell sequencing
      ↓
Coverage waypoints
```

The planner also uses obstacle inflation before decomposition. This is important because the UAV should not plan directly along obstacle boundaries; a safety margin should exist between the vehicle and obstacles.

---

# 7. Cell Decomposition

The grid-based implementation sweeps through the environment column by column.

For each column, contiguous free-space segments are identified. These segments are then matched with free-space segments in the previous column.

Conceptually:

```text
Column i-1       Column i

   FREE              FREE
   FREE              FREE
   FREE              FREE
   ████              ████
   ████              ████
   FREE              FREE
   FREE              FREE
```

If the free-space segments overlap, they are treated as belonging to the same cell.

This provides a discrete approximation of the classical BCD process.

---

# 8. Obstacle Inflation

Obstacle inflation is included to increase the safety margin around occupied cells.

Without inflation:

```text
UAV path
───────────────
      █████
      █████
      █████
```

With inflation:

```text
        SAFE MARGIN
    ┌───────────────┐
    │    █████      │
    │    █████      │
    │    █████      │
    └───────────────┘
```

This is particularly important for UAV operation because localization error, controller tracking error, vehicle dimensions, and simulation/real-world discrepancies can make a path that is theoretically collision-free unsafe in practice.

---

# 9. Coverage Path Generation

After decomposition, the planner generates a lawnmower path for each cell.

The row spacing is configurable and determines the distance between adjacent coverage sweeps.

A smaller spacing produces:

* denser coverage,
* greater path length,
* greater mission duration,
* potentially greater energy consumption.

A larger spacing produces:

* fewer waypoints,
* shorter flight time,
* lower computational and execution cost,
* but potentially uncovered regions.

Therefore, row spacing is directly related to the UAV's sensor footprint and required coverage resolution.

The paths generated inside individual cells are subsequently ordered using a nearest-neighbor strategy.

---

# 10. Cell Sequencing

Once individual cell paths have been generated, the planner selects the next cell using a greedy nearest-neighbor strategy.

The basic procedure is:

```text
Start Cell
    │
    ▼
Find nearest unvisited cell
    │
    ▼
Fly coverage path
    │
    ▼
Find next nearest cell
    │
    ▼
Repeat
```

This provides a computationally simple way of ordering the cells.

However, it is important to distinguish **coverage completeness** from **route optimality**. The nearest-neighbor method provides a practical ordering but does not guarantee the globally shortest route.

---

# 11. Waypoint Generation

The final coverage trajectory is exported as a CSV file.

The current planner generates local coordinates containing:

```text
sequence
x_local
y_local
z
```

The mission executor then loads these local waypoints and converts them to geographic coordinates using the UAV's home position.

This allows the coverage planner to remain independent of the UAV's absolute GPS position.

---

# 12. UAV Mission Execution

The UAV execution stage uses MAVSDK to communicate with the simulated vehicle.

The current mission executor performs the following sequence:

```text
1. Connect to UAV
       ↓
2. Wait for valid global position/home
       ↓
3. Load coverage waypoints
       ↓
4. Convert local coordinates to global coordinates
       ↓
5. Construct mission
       ↓
6. Add TAKEOFF command
       ↓
7. Add coverage waypoints
       ↓
8. Add RTL command
       ↓
9. Upload mission
       ↓
10. Arm
       ↓
11. Take off
       ↓
12. Switch to AUTO
       ↓
13. Execute coverage mission
       ↓
14. Log GPS trajectory
       ↓
15. Complete mission / RTL
```

The current repository explicitly constructs a takeoff item, coverage waypoint items, and a Return-to-Launch item before uploading the mission.

The executor also waits for a valid connection and global/home position before constructing the mission, which reduces the likelihood of starting with invalid geographic coordinates.

---

# 13. Autonomous Flight and Telemetry Logging

During mission execution, the UAV's position is continuously recorded.

The actual trajectory is stored as:

```text
timestamp_s
latitude
longitude
altitude
```

The repository's mission executor creates `actual_path.csv` and records the UAV's telemetry position while the mission is running.

This is a particularly important component of the project because it allows the planner to be evaluated using the **actual flight trajectory**, rather than only the theoretically generated path.

---

# 14. Planned-versus-Actual Trajectory Evaluation

The planned path is represented in local East-North coordinates.

The actual GPS trajectory is converted from:

```text
Latitude / Longitude
```

to:

```text
East / North coordinates in metres
```

using the home position as the local reference.

The resulting visualization contains:

```text
Planned trajectory
        vs.
Actual UAV trajectory
```

The repository already contains a dedicated `path_plotter.py` implementation for this purpose. It loads the planned waypoint CSV and actual GPS CSV, converts the actual GPS track into local coordinates, and generates a `planned_vs_actual.png` figure.

This plot should be included as one of the primary experimental results.

---

# 15. Evaluation Metrics

The single-UAV implementation should be evaluated using the following metrics.

## 15.1 Coverage Completeness

The percentage of the free area that was successfully covered.

[
Coverage\ Rate =
\frac{A_{covered}}{A_{free}}\times100
]

A successful mission should achieve coverage as close as possible to 100% of the intended accessible region, subject to the chosen sensor footprint and obstacle clearance.

---

## 15.2 Path Length

The total distance traveled by the UAV:

[
L = \sum_{i=1}^{N-1}
\sqrt{(x_{i+1}-x_i)^2+(y_{i+1}-y_i)^2}
]

This can be calculated independently for:

* planned trajectory,
* actual trajectory.

The difference indicates how closely the UAV followed the intended path.

---

## 15.3 Tracking Error

The deviation between the actual and planned trajectories can be measured using:

* mean tracking error,
* maximum tracking error,
* RMSE,
* endpoint error.

A useful metric is:

[
RMSE =
\sqrt{
\frac{1}{N}
\sum_{i=1}^{N}
e_i^2
}
]

where (e_i) represents the distance between the actual UAV position and the corresponding planned position.

---

## 15.4 Mission Duration

Total time required to complete the coverage mission.

This is important because UAV coverage performance is constrained by:

* battery,
* flight time,
* speed,
* number of turns,
* transition distance between cells.

---

## 15.5 Number of Waypoints

The total number of generated waypoints should also be recorded.

A very large number of waypoints may increase:

* mission upload size,
* autopilot processing,
* execution time,
* path-following overhead.

---

# 16. Experimental Validation

The final experiment should demonstrate the complete pipeline in Gazebo.

The recommended validation sequence is:

```text
Gazebo environment
       ↓
ROS2 map available
       ↓
Coverage planner
       ↓
waypoints.csv
       ↓
SITL UAV
       ↓
Autonomous takeoff
       ↓
Coverage mission
       ↓
RTL
       ↓
actual_path.csv
       ↓
planned_vs_actual.png
```

The experiment should record at least:

| Metric                 | Result           |
| ---------------------- | ---------------- |
| Map size               | To be recorded   |
| Map resolution         | To be recorded   |
| Number of cells        | To be recorded   |
| Number of waypoints    | To be recorded   |
| Planned path length    | To be calculated |
| Actual path length     | To be calculated |
| Mission duration       | To be recorded   |
| Maximum tracking error | To be calculated |
| RMSE tracking error    | To be calculated |
| Coverage percentage    | To be calculated |
| Successful RTL         | Yes/No           |

The final report should replace the "To be recorded" fields with measurements from the successful Gazebo run.

---

# 17. Planned vs Actual Flight Path

The expected final plot should look conceptually like:

<img width="1279" height="1048" alt="planned_vs_actual" src="https://github.com/user-attachments/assets/66e87b5b-8bc5-4311-9d44-d92b49d56629" />

The planned trajectory represents the output of the BCD planner, while the actual trajectory represents the UAV's recorded GPS motion.

The two paths will not necessarily overlap perfectly because of:

* waypoint acceptance radius,
* UAV dynamics,
* GPS noise,
* controller response,
* acceleration/deceleration,
* mission execution behavior.

Therefore, a small difference between the paths does not necessarily indicate planner failure.

The repository's plotting implementation already produces a dedicated "Boustrophedon Coverage – Planned vs Actual" visualization.

---

# 18. Weaknesses and Edge Cases

At least two weaknesses should be explicitly documented because they become significantly more important when extending the system from one UAV to a swarm.

## 18.1 Weakness 1 — Grid Resolution and Coverage-Line Spacing

The grid-based planner is inherently dependent on the occupancy-grid resolution.

A cell boundary represented at coarse resolution may not accurately represent the true obstacle boundary.

For example:

```text
Actual obstacle:

      ███████
      ███████

Coarse grid:

      ██████
      ██████
```

Similarly, the coverage path is generated using a configurable row spacing. If the spacing is larger than the effective sensor footprint, portions of the environment may not be observed.

This creates a fundamental trade-off:

```text
Smaller spacing
     ↓
Better coverage
     ↓
Longer flight
     ↓
Higher energy consumption

Larger spacing
     ↓
Shorter flight
     ↓
Lower energy consumption
     ↓
Potential coverage gaps
```

### Future multi-agent consideration

For a swarm, inconsistent or poorly selected coverage spacing could cause two UAVs to either:

* leave uncovered regions, or
* repeatedly cover the same region.

A future multi-UAV planner should therefore explicitly model the UAV/sensor footprint and use a coverage-resolution parameter derived from that footprint rather than treating row spacing only as a geometric parameter.

---

# 19. Weakness 2 — Greedy Cell Sequencing Does Not Guarantee Optimal Transitions

The current implementation uses a nearest-neighbor strategy to select the next cell.

This is computationally simple, but it is not globally optimal.

A locally closest cell may lead to a poor overall route:

```text
        Cell B
          ↑
          │
Start → Cell A

Cell C ────────────────┐
                       │
                       ↓
                    Cell D
```

The nearest-neighbor decision at one stage can produce a substantially longer total mission later.

More importantly, the transition between cells is currently treated as a straight-line connection.

A future implementation should verify that this transition is collision-free instead of assuming that the straight line between two cell endpoints is always safe.

### Future multi-agent consideration

This becomes much more important with multiple UAVs.

A swarm planner needs to optimize not only:

* distance between cells,

but also:

* UAV assignment,
* workload balance,
* transition distance,
* collision avoidance,
* synchronization,
* battery consumption.

A nearest-neighbor single-UAV heuristic therefore cannot simply be replicated independently for every UAV.

---

# 20. Weakness 3 — No Explicit Multi-UAV Conflict Resolution

The current implementation is designed around a single vehicle.

There is no mechanism that prevents two future UAVs from being assigned overlapping regions or crossing trajectories.

For example:

```text
UAV 1
───────────────→
              X
              ←───────────────
                         UAV 2
```

The intersection point `X` represents a potential conflict.

### Future multi-agent consideration

A swarm extension will require:

* exclusive cell assignment,
* inter-UAV separation constraints,
* trajectory conflict detection,
* priority rules,
* temporal scheduling,
* replanning after UAV failure.

The decomposition stage should therefore eventually produce a representation that can be partitioned among multiple UAVs.

---

# 21. Weakness 4 — Single Point of Failure

The current architecture assumes one UAV is responsible for the entire mission.

If that UAV:

* loses communication,
* runs out of battery,
* experiences a navigation error,
* fails to reach a waypoint,

then the remaining uncovered area is not automatically reassigned.

### Future multi-agent consideration

A swarm should support dynamic task redistribution.

For example:

```text
UAV 1 fails
     │
     ▼
Detect failure
     │
     ▼
Identify uncovered cells
     │
     ▼
Reassign cells
     │
     ├──────► UAV 2
     │
     └──────► UAV 3
```

This is one of the major advantages of moving from a single-UAV architecture toward a coordinated swarm.

---

# 22. Weakness 5 — Coordinate Conversion Assumptions

The current mission pipeline converts local coordinates into global latitude/longitude using a local flat-earth approximation.

This is appropriate for relatively small simulated environments, but the approximation becomes less suitable as the operational area grows.

The path plotting component also uses a fixed home latitude and longitude in its current configuration.

For the final experiment, the home position used for plotting must therefore match the actual SITL home position.

For larger environments or real-world deployment, a proper local projected coordinate system should be used consistently throughout planning, execution, and evaluation.

---

# 23. Implications for Future Multi-UAV Swarm Extension

The single-UAV implementation establishes the fundamental building blocks needed for a swarm:

```text
Map
 ↓
Decomposition
 ↓
Coverage cells
 ↓
Cell paths
 ↓
UAV assignment
 ↓
Mission execution
```

The major new problem is the **cell allocation layer**.

Instead of:

```text
All cells
   ↓
One UAV
```

the future system should perform:

```text
All cells
   ↓
Task allocation
   ├───────────┐
   ▼           ▼
 UAV 1       UAV 2       ... UAV N
   │           │
   ▼           ▼
Cell set 1   Cell set 2
```

The allocation objective should consider more than equal numbers of cells.

A better objective would consider:

[
J =
w_1L +
w_2T +
w_3E +
w_4O +
w_5C
]

where:

* (L) = flight distance,
* (T) = mission time,
* (E) = estimated energy consumption,
* (O) = coverage overlap,
* (C) = collision/conflict cost.

This would allow future work to move from simple BCD coverage toward coordinated multi-UAV coverage optimization.

---

# 24. Recommended Future Architecture

A scalable swarm architecture can be structured as:

```text
                 Global Map
                     │
                     ▼
             BCD Decomposition
                     │
                     ▼
              Coverage Cells
                     │
                     ▼
            Task Allocation Layer
                     │
       ┌─────────────┼─────────────┐
       ▼             ▼             ▼
     UAV 1         UAV 2         UAV N
       │             │             │
       ▼             ▼             ▼
 Local Planner   Local Planner   Local Planner
       │             │             │
       ▼             ▼             ▼
   Controller     Controller     Controller
       │             │             │
       └─────────────┼─────────────┘
                     ▼
             Shared State / Map
```

A future system could investigate algorithms such as:

* auction-based task allocation,
* market-based allocation,
* Hungarian assignment,
* Voronoi-based partitioning,
* DARP-style area partitioning,
* MILP optimization,
* distributed task allocation,
* consensus-based coordination.

Multi-robot coverage literature demonstrates that partitioning an area among robots is a central challenge in multi-agent coverage planning. For example, DARP explicitly focuses on dividing a terrain into regions corresponding to individual robots while seeking complete and efficient coverage.

---

# 25. Why BCD Remains a Good Foundation

BCD is a useful foundation because decomposition separates a difficult global coverage problem into smaller regions.

Instead of asking:

> "How does the UAV cover this entire complex environment?"

the planner asks:

> "How can the environment be divided into simpler cells that can each be covered systematically?"

This separation is especially useful for swarm extension.

Once the cells have been generated, they can become **tasks**.

For example:

```text
Cell 0 ──► UAV 1
Cell 1 ──► UAV 2
Cell 2 ──► UAV 1
Cell 3 ──► UAV 3
Cell 4 ──► UAV 2
```

Thus, the BCD decomposition can serve as the interface between:

**geometric coverage planning**

and

**multi-agent task allocation.**

---

# 26. Expected Final Demonstration

The final single-UAV demonstration should show the following sequence:

### Stage 1 — Environment

Launch Gazebo with the selected coverage environment and obstacles.

### Stage 2 — Mapping

Confirm that the occupancy grid is available through ROS2.

### Stage 3 — Planning

Run the BCD planner and verify:

* map received,
* obstacles recognized,
* cells generated,
* coverage path generated,
* waypoint CSV created.

### Stage 4 — Visualization

Open RViz2 and verify:

* map,
* BCD cells,
* coverage path.

### Stage 5 — Mission Execution

Launch the UAV mission executor.

Verify:

* MAVSDK connection,
* valid position/home,
* arming,
* takeoff,
* AUTO mode,
* coverage waypoint execution,
* RTL.

The current mission executor explicitly implements these mission stages and records the actual GPS track.

### Stage 6 — Evaluation

After mission completion:

```text
waypoints.csv
       +
actual_path.csv
       ↓
path_plotter.py
       ↓
planned_vs_actual.png
```

The generated figure should then be included as the primary experimental result.

---

# 27. Expected Deliverables

The completed single-UAV milestone should contain:

### Software

* BCD coverage planner
* ROS2 integration
* obstacle handling
* waypoint generation
* MAVSDK mission executor
* autonomous takeoff
* autonomous coverage
* RTL
* GPS logging
* trajectory plotting

### Data

* `waypoints.csv`
* `actual_path.csv`

### Visualization

* BCD cell visualization
* planned coverage trajectory
* actual UAV trajectory
* final planned-vs-actual plot

### Documentation

* system architecture
* algorithm description
* experimental setup
* evaluation metrics
* weaknesses/edge cases
* future multi-UAV considerations

The repository already contains the core software organization for these outputs, including the mission executor and path plotting components.

---

# 28. Conclusion

This work developed a complete foundation for autonomous single-UAV area coverage using Boustrophedon Cellular Decomposition. The approach transforms an obstacle-containing environment into a collection of simpler coverage cells and generates systematic lawnmower trajectories within those cells. The generated local waypoints are subsequently converted into geographic coordinates and executed by an autonomous UAV mission in simulation.

The current implementation provides a practical bridge between coverage-path planning and UAV autonomy: the planner produces the mission, the mission executor uploads and executes it, telemetry is recorded during flight, and the actual trajectory can be compared against the intended trajectory. The repository contains the corresponding planning, mission-execution, and path-plotting components.

The planned-versus-actual trajectory comparison is particularly important because it validates the complete system rather than only the planning algorithm. A successful Gazebo experiment should demonstrate that the UAV can autonomously take off, follow the generated BCD coverage trajectory, complete the intended coverage region, return safely to the launch location, and produce an actual flight track that follows the planned path with an acceptable tracking error.

Several limitations were identified. Grid resolution and coverage spacing can influence coverage completeness; greedy cell sequencing does not guarantee an optimal route and requires safe transition validation; and the current architecture does not provide multi-UAV conflict resolution or dynamic task reassignment. These limitations are not merely implementation issues—they define the major research challenges for extending the system from a single UAV to a coordinated swarm.

Consequently, the single-UAV implementation should be considered the **foundation layer for future multi-agent coverage**. The next stage can build upon the existing BCD cells by introducing task allocation, workload balancing, inter-UAV collision avoidance, communication, dynamic replanning, and failure recovery. This progression provides a clear path from a working single-agent coverage mission toward a scalable autonomous UAV swarm coverage system.

---

## Repository

The implementation and associated source files are maintained in the project repository:

[Boustrophedon Coverage Algorithm with Obstacles — GitHub](https://github.com/oye-ahmad/Boustrophedon-Coverage-Algorithm-with-obstacles?utm_source=chatgpt.com)

The repository currently contains the `src` components for the coverage planner, MAVSDK waypoint following/mission execution, and planned-versus-actual trajectory plotting.

## References

1. Choset, H., and Pignon, P. *Coverage Path Planning: The Boustrophedon Cellular Decomposition*. Proceedings of the International Conference on Field and Service Robotics, 1998.

2. The project repository: *Boustrophedon Coverage Algorithm with Obstacles*.

3. The project's mission executor implements autonomous takeoff, coverage waypoint upload, AUTO-mode execution, RTL, and GPS trajectory logging.

4. The project's path-plotting component converts the actual GPS trajectory to local East-North coordinates and generates the planned-versus-actual coverage plot.

5. Fields2Cover provides an example of modern coverage-planning systems combining decomposition, route optimization, and coverage planning, illustrating directions for future optimization work.

Now this same concept will be applied to multiple UAVs
