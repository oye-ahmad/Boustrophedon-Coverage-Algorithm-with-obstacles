
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
