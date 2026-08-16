Here is the Graph of the Path of BCD Algorithm

**BCD Geometric-Based**

<img width="2537" height="2562" alt="coverage_plan" src="https://github.com/user-attachments/assets/19173e6f-b68f-4ca7-a6b3-bbdf6244ef9c" />

**BCD Grid-Based**

<img width="1279" height="1048" alt="planned_vs_actual" src="https://github.com/user-attachments/assets/66e87b5b-8bc5-4311-9d44-d92b49d56629" />

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

Geometric Boustrophedon Cellular Decomposition
1. Overview

The geometric implementation is a more mathematically oriented version of Boustrophedon Cellular Decomposition. Instead of representing the environment as a discrete occupancy grid, it represents the workspace and obstacles as continuous geometric polygons using the Shapely computational-geometry library.

The fundamental idea is:

Move a vertical sweep line through the environment and observe how the connectivity of free space changes.

Whenever the topology of the free space changes, a new BCD cell is created, terminated, split, or merged.

The complete process is:

Boundary + Obstacles
        │
        ▼
 Critical X Coordinates
        │
        ▼
     Sweep Line
        │
        ▼
 Free-space intervals
        │
        ▼
 Split / Merge / Continuation Events
        │
        ▼
 Geometric BCD Cells
        │
        ▼
 Polygonal Cell Representation
        │
        ▼
 Lawn-mower Coverage
        │
        ▼
 Nearest-neighbor Cell Sequencing
        │
        ▼
 Ordered Coverage Path
        │
        ├──────────────► ROS2 /coverage_path
        │
        ├──────────────► /bcd_cells
        │
        ├──────────────► waypoints.csv
        │
        └──────────────► coverage_plan.png
2. Why is it called "Geometric BCD"?

The key difference is the representation of the environment.

The grid-based implementation sees the world as:

0 0 0 0 0 0
0 0 1 1 0 0
0 0 1 1 0 0
0 0 0 0 0 0

where individual cells represent free and occupied space.

The geometric implementation instead sees:

Boundary
┌───────────────────────────────┐
│                               │
│      ┌───────────┐            │
│      │  obstacle │            │
│      └───────────┘            │
│                               │
└───────────────────────────────┘

The boundaries are represented using actual coordinates such as:

(15,30)
(25,30)
(25,70)
(15,70)

Therefore, the algorithm operates directly in continuous x,y space.

3. Environment Representation

Your geometric planner defines a 100 m × 100 m workspace:

self.boundary = sg.Polygon([
    (0, 0),
    (100, 0),
    (100, 100),
    (0, 100)
])

So the workspace is:

y
↑
100 ┌──────────────────────────┐
    │                          │
    │                          │
    │                          │
    │                          │
    │                          │
  0 └──────────────────────────┘ → x
    0                         100

This is a continuous geometric environment rather than a raster map.

4. Obstacles

The implementation defines three obstacles.

Obstacle A
self.obstacle_a = sg.Polygon([
    (15,30),
    (25,30),
    (25,70),
    (15,70)
])

This is a vertical rectangular obstacle.

Conceptually:

       x=15       x=25
         │         │
         ┌─────────┐
         │         │
         │         │
         │         │
         │         │
         └─────────┘

When the sweep line reaches this obstacle, the free space changes.

5. Obstacle B

The second obstacle is L-shaped:

self.obstacle_b = sg.Polygon([
    (40,20),
    (60,20),
    (60,40),
    (50,40),
    (50,80),
    (40,80)
])

Its shape is approximately:

             ┌─────┐
             │     │
             │     │
             │     │
┌────────────┘     │
│                  │
│                  │
└──────────────────┘

This obstacle is useful because it creates more complicated changes in free-space connectivity.

6. Obstacle C

The third obstacle is U-shaped:

self.obstacle_c = sg.Polygon([
    (75,20),
    (90,20),
    (90,80),
    (75,80),
    (75,60),
    (83,60),
    (83,40),
    (75,40)
])

This gives the planner another topology-changing structure.

The three obstacles were intentionally selected to demonstrate different BCD events.

7. The Sweep-Line Concept

This is the heart of geometric BCD.

Imagine a vertical line:

      │
      │
      │
      │
      │
      │
      │

The algorithm moves this line from:

x = 0

toward:

x = 100

At every position, it asks:

"What portions of this vertical line are free space?"

For example:

      │
 FREE │
      │
──────│────── obstacle
      │
      │
 FREE │
      │

The free parts become free-space intervals.

8. Critical X Coordinates

The algorithm does not need to evaluate every possible x-coordinate.

Instead, it finds the x-coordinates where geometry can change.

This is implemented using:

x_coords = set()

and then extracting coordinates from:

self.boundary

and:

self.obstacles

For your environment, the critical x-coordinates are approximately:

0
15
25
40
50
60
75
83
90
100

These coordinates correspond to obstacle and boundary vertices.

The algorithm then sorts them:

sorted_x = sorted(list(x_coords))
9. Why Critical Coordinates Matter

Between two consecutive critical coordinates, the topology of the environment does not normally change.

For example:

15              25
│                │
│    obstacle    │
│                │

Within:

15<x<25

the obstacle structure remains the same.

Therefore, the algorithm evaluates a representative position:

x
mid
	​

=
2
x
curr
	​

+x
next
	​

	​


implemented as:

x_mid = (x_curr + x_next) / 2.0

This is a clever technique because it avoids directly intersecting the sweep line with obstacle vertices.

10. Finding Free-Space Intervals

The function:

_get_free_intervals_at(x)

is responsible for determining which portions of the sweep line are free.

It creates:

vertical_line = sg.LineString([
    (x, -10.0),
    (x, 110.0)
])

This line passes through the entire workspace.

Then it intersects the line with the boundary.

Next, it intersects the line with every obstacle.

Suppose the sweep line intersects an obstacle between:

y=30

and:

y=70

Then:

Sweep line


y=100 ─── FREE
         │
y=70  ───┤
         │
         │ OBSTACLE
         │
y=30  ───┤
         │
y=0   ─── FREE

The free-space intervals become:

[0,30]

and:

[70,100]
11. Computing the Free-Space Complement

The implementation first collects occupied intervals:

occupied_segments

Then sorts them:

occupied_segments.sort(...)

and merges overlapping obstacle intervals.

Finally, it computes their complement within the workspace.

Conceptually:

Boundary:
0 ───────────────────────── 100


Obstacle:
       30 ─────── 70


Free:
0 ─── 30          70 ─── 100

So the algorithm gets:

[
    (0,30),
    (70,100)
]

These are the free-space intervals at that particular x-coordinate.

12. Cell Tracking

Now comes the actual BCD decomposition.

The algorithm maintains:

active_cells

These are cells that currently exist at the sweep position.

Suppose initially the sweep line sees:

ONE FREE INTERVAL

Then:

        │
        │
        │
        │
        │

There is one active cell.

If later the obstacle causes the free space to split:

        │
 FREE   │
        │
──── OBSTACLE ────
        │
 FREE   │
        │

then one cell becomes two.

This is a split event.

13. Continuation Event

Suppose:

Before:


     FREE
     FREE
     FREE


After:


     FREE
     FREE
     FREE

The connectivity has not changed.

The same cell continues.

The implementation associates the new interval with the existing cell using overlap.

14. Split Event

A split occurs when:

one previous interval
        ↓
multiple current intervals

Conceptually:

Before sweep:


     ┌─────────┐
     │  Cell A │
     │         │
     └─────────┘


After obstacle:


     ┌────┐   ┌────┐
     │ B  │   │ C  │
     │    │   │    │
     └────┘   └────┘

Therefore:

A→B+C

The implementation attempts to detect this through:

split_events

and creates a new Cell for the new interval.

15. Merge Event

The reverse happens during a merge:

Before:


     ┌────┐   ┌────┐
     │ A  │   │ B  │
     │    │   │    │
     └────┘   └────┘


After:


       ┌─────────┐
       │    C    │
       │         │
       └─────────┘

Two free-space components become one.

The BCD interpretation is:

A+B→C

The implementation closes the corresponding cells and stores them in:

closed_cells
16. The Cell Representation

Each geometric cell stores:

self.slices

where each slice is:

(x, y_bottom, y_top)

For example:

(10, 0, 100)
(11, 0, 100)
(12, 0, 100)
(13, 20, 100)
(14, 20, 100)

This describes how the cell changes as x increases.

The cell eventually gets converted into an actual Shapely polygon.

17. Constructing the Cell Polygon

When a cell is closed:

cell.close(...)

the algorithm sorts the slices by x.

It creates:

bottom boundary

from left to right:

bottom_pts

and:

top boundary

from right to left:

top_pts

These are combined into one polygon.

Conceptually:

Bottom boundary
→ → → → → →


← ← ← ← ← ←
Top boundary

Together they form a closed polygon.

18. Shapely Validation

The implementation creates:

raw_poly = sg.Polygon(clean_points)

and checks:

raw_poly.is_valid

If the polygon is invalid, it tries:

raw_poly = raw_poly.buffer(0)

This is a common Shapely technique for attempting to repair certain invalid geometries.

The resulting object is stored in:

cell.polygon

Therefore, unlike the grid implementation, each cell is now a genuine geometric region.

19. Why This Is More Powerful Than the Grid Version

Consider an obstacle boundary at:

x=23.73

A geometric planner can represent it exactly.

A grid planner with 0.1 m resolution can only approximate it using grid cells.

Therefore:

Grid BCD
Continuous environment
        ↓
Discretization
        ↓
Grid
        ↓
Approximate geometry
Geometric BCD
Continuous environment
        ↓
Geometric processing
        ↓
Continuous polygons

This makes the geometric approach attractive for higher-precision planning.

20. Coverage Generation Inside the Geometric Cells

Once the cells have been created, the planner generates the actual coverage trajectory.

It first obtains the polygon bounds:

min_x, min_y, max_x, max_y = cell.polygon.bounds

Then it creates horizontal sweep lines.

The spacing is:

self.row_spacing = 3.0

so the lines are approximately:

y = ...
y = ...
y = ...
y = ...

with 3 m separation.

21. Polygon-Line Intersection

This is one of the strongest parts of your geometric implementation.

For each sweep line:

horizontal_line = sg.LineString(...)

the algorithm computes:

intersection = cell.polygon.intersection(horizontal_line)

Suppose the cell is irregular:

       ┌───────────────┐
       │               │
       │       ┌───────┘
       │       │
       └───────┘

A horizontal sweep may intersect it differently at different y-values.

The geometric planner automatically calculates the valid portion.

So the path conforms to the actual cell geometry.

22. Boustrophedon Motion

The planner alternates direction.

For example:

Sweep 1:  ─────────────→
Sweep 2:  ←─────────────
Sweep 3:  ─────────────→
Sweep 4:  ←─────────────

This is exactly where the name Boustrophedon comes from.

"Boustrophedon" roughly refers to the way an ox turns while ploughing a field.

The implementation uses:

reverse = False

and alternates it after each sweep.

23. Why Polygon Intersection Matters

Suppose the sweep line intersects a cell only from:

x=20

to:

x=43

The UAV gets:

(20,y) → (43,y)

instead of blindly traveling across the entire bounding box.

This prevents the path from entering regions outside the actual cell.

That is a major advantage over a naive rectangular lawnmower planner.

24. Cell-to-Cell Sequencing

After generating a path for each cell, the planner uses nearest-neighbor sequencing.

It starts with the first cell:

current_id = unvisited.pop(0)

Then it examines every remaining cell and computes:

dist = np.linalg.norm(
    last_pt - cand_start
)

The nearest cell becomes the next cell.

So:

Cell 1
  ↓
nearest
  ↓
Cell 4
  ↓
nearest
  ↓
Cell 2
  ↓
nearest
  ↓
Cell 3

This generates one continuous ordered coverage path.

25. Geometric BCD Output

The planner produces several outputs.

ROS2 path
/coverage_path

containing:

x
y
z

for each waypoint.

Cell visualization
/bcd_cells

which allows the decomposition to be viewed in RViz2.

CSV

The planner creates:

waypoints.csv

containing:

seq
x_local
y_local
z
latitude
longitude
Debug plot

It also creates:

coverage_plan.png

showing:

workspace boundary,
obstacles,
BCD cells,
cell labels,
coverage path,
start point,
end point.

This makes the geometric implementation particularly useful for algorithm validation.

26. Geometric BCD + GPS

Your implementation also contains:

class ENUToGeodeticConverter:

This converts local:

(x,y,z)

coordinates into:

(latitude,longitude,altitude)

coordinates.

The idea is:

Local ENU
  │
  │ x,y,z
  ▼
Projection
  │
  ▼
Latitude/Longitude

This is useful because your geometric planner can produce a path in meters while the UAV mission can ultimately operate using geographic coordinates.

However, your current implementation uses:

map_origin_lat = 37.7749
map_origin_lon = -122.4194

which corresponds to the San Francisco area.

For your actual Gazebo/ArduPilot experiment, this should be replaced by the actual mission origin/home position if GPS coordinates from this planner are used.

27. Geometric BCD vs Grid BCD

The comparison can now be stated more precisely.

Property	Grid BCD	Geometric BCD
Representation	Occupancy cells	Continuous polygons
Input	ROS2 /map	Boundary + obstacle polygons
Sweep	Grid columns	Critical x-coordinate intervals
Cell	Grid slices	Shapely polygon
Resolution dependency	Yes	No
Obstacle boundary accuracy	Limited by resolution	High
Obstacle inflation	Implemented	Not currently implemented
Split/merge modeling	Approximate	Explicitly attempted
Path generation	Grid rows	Polygon-line intersection
Irregular cells	Limited	Strong
ROS map compatibility	Direct	Requires conversion
GPS conversion	Not included	Included
Debug plot	Not inherent	Included
Computational complexity	Lower	Higher
Geometric precision	Lower	Higher
Best use	Real ROS occupancy maps	Algorithmic/geometric planning
28. The Most Important Weakness in Your Geometric Implementation

There is one issue I would explicitly mention in your report.

The code comments say:

"""Implements Choset's exact sweep-line BCD algorithm."""

I would not use the word "exact" in the final academic report yet.

Why?

Because your event matching is based primarily on maximum interval overlap:

overlap = max(
    0.0,
    min(interval[1], last_slice[2])
    - max(interval[0], last_slice[1])
)

and:

best_cell = ...

This is a heuristic correspondence mechanism.

A rigorous geometric BCD implementation should explicitly construct the connectivity relationship between:

S
i−1
	​


and:

S
i
	​


and then identify:

continuation,
split,
merge,
appearance,
disappearance.

Your implementation is geometric and strongly inspired by classical BCD, but the event correspondence should be strengthened before describing it as a mathematically exact implementation.

A better statement for the report is:

"The implementation follows the sweep-line formulation of geometric Boustrophedon Cellular Decomposition and uses interval-overlap analysis to identify cell continuation, split, and merge events."

That is accurate and defensible.

29. Another Geometric BCD Edge Case

Consider two intervals:

Previous:


A: ───────────────




Current:


B: ───────
C: ───────

The algorithm needs to recognize:

A→B+C

But your _match_intervals() function independently assigns each new interval to the cell with maximum overlap.

This can lead to ambiguous assignments in complicated environments.

For example:

Previous:


A ───────────────
B ───────────────




Current:


C ───────────────

The algorithm needs to recognize:

A+B→C

rather than simply assigning C to whichever cell has slightly greater overlap.

This becomes increasingly important with complicated obstacle configurations.

30. Another Edge Case: Narrow Passages

Suppose:

┌──────────────┐
│              │
│   obstacle   │
│   ┌──────┐   │
│   │      │   │
│   └──┐   │   │
│      │   │   │
└──────┴───┴───┘

There may be a very narrow passage.

A geometric planner can represent the passage, but the UAV may physically be unable to safely fly through it.

This means geometric free space is not necessarily equivalent to UAV-feasible free space.

The solution is geometric obstacle inflation:

O
′
=O⊕B(r
safe
	​

)

or, in Shapely terms, something like:

inflated_obstacle = obstacle.buffer(safety_distance)

This should be added before decomposition.

31. Another Edge Case: Very Small Cells

Suppose decomposition creates:

Cell 1: 50 m²
Cell 2: 0.4 m²
Cell 3: 0.7 m²

The UAV may spend more time transitioning to tiny cells than actually covering them.

A future implementation should therefore consider a minimum cell area:

A
cell
	​

>A
min
	​


and potentially merge or discard cells that are too small to be practically useful.

This becomes even more important in multi-UAV operation.

32. Another Edge Case: Coverage Path Through Cell Transitions

Your cell sequencing finds the nearest cell, but the transition between cells is essentially assumed to be valid.

For example:

Cell A                  Cell B


████████                ████████
████████                ████████
████████                ████████
        \              /
         \            /
          \          /

The straight transition could potentially cross an obstacle.

Therefore, a robust implementation should perform:

Cell A endpoint
       ↓
Collision checking
       ↓
Safe transition planner
       ↓
Cell B start

rather than directly joining the two endpoints.

33. How Geometric BCD Fits Your Overall UAV Project

Your two implementations actually complement each other very well.

You can think of them as:

Grid BCD

Engineering implementation

Gazebo
 ↓
ROS /map
 ↓
OccupancyGrid
 ↓
Grid BCD
 ↓
UAV
Geometric BCD

Algorithmic/research implementation

Polygon environment
 ↓
Geometric BCD
 ↓
Exact-ish cell geometry
 ↓
Coverage trajectory
 ↓
Analysis / visualization

The geometric implementation is therefore excellent for explaining how BCD works mathematically, while the grid implementation is better for demonstrating integration with the actual ROS2/Gazebo UAV environment.

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
