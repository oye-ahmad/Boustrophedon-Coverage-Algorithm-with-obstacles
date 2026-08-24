#!/usr/bin/env python3
"""
plot_combined_coverage.py

Extends your path_plotter.py to all 3 drones, and fixes an existing bug
along the way: your original path_plotter.py read actual_path.csv using
columns 'lat'/'lon', but MAVSDKWaypointFollower._log_telemetry() (and its
multi-drone version) actually write 'latitude'/'longitude' - that mismatch
throws a KeyError. This version reads the correct column names.

Reads, per drone:
  waypoints_droneN.csv     (columns: seq, x_local_m, y_local_m, z_m)
  actual_path_droneN.csv   (columns: timestamp, latitude, longitude,
                             abs_altitude, rel_altitude)
and produces one combined planned-vs-actual coverage map, plus prints the
same per-drone range summary your original script printed.
"""

import csv
import math
import os
import matplotlib.pyplot as plt

WAYPOINT_DIR = os.path.expanduser("~/coverage_map")
EARTH_RADIUS_M = 6378137.0

# Must match the HOME_LAT/HOME_LON your SITL instances actually use.
HOME_LAT = -35.3632622
HOME_LON = 149.1652375

N_DRONES = 3
DRONE_COLORS = {1: "tab:blue", 2: "tab:orange", 3: "tab:green"}


def load_planned(drone_id: int):
    path = os.path.join(WAYPOINT_DIR, f"waypoints_drone{drone_id}.csv")
    xs, ys = [], []
    with open(path) as f:
        for row in csv.DictReader(f):
            xs.append(float(row["x_local_m"]))
            ys.append(float(row["y_local_m"]))
    return xs, ys


def load_actual(drone_id: int, home_lat: float, home_lon: float):
    """Convert lat/lon -> local East-North metres (flat-earth), same
    formula as your original path_plotter.py, but reading the CORRECT
    column names ('latitude'/'longitude', not 'lat'/'lon')."""
    path = os.path.join(WAYPOINT_DIR, f"actual_path_drone{drone_id}.csv")
    xs, ys = [], []
    with open(path) as f:
        for row in csv.DictReader(f):
            lat = float(row["latitude"])
            lon = float(row["longitude"])
            d_lat = math.radians(lat - home_lat)
            d_lon = math.radians(lon - home_lon)
            x = d_lon * EARTH_RADIUS_M * math.cos(math.radians(home_lat))
            y = d_lat * EARTH_RADIUS_M
            xs.append(x)
            ys.append(y)
    return xs, ys


def main():
    fig, ax = plt.subplots(figsize=(10, 8))

    for drone_id in range(1, N_DRONES + 1):
        color = DRONE_COLORS.get(drone_id, "gray")

        try:
            px, py = load_planned(drone_id)
            print(f"Drone {drone_id} planned points: {len(px)}  "
                  f"x range: {min(px):.2f} -> {max(px):.2f} m  "
                  f"y range: {min(py):.2f} -> {max(py):.2f} m")
            ax.plot(px, py, linestyle="-", marker="o", markersize=3,
                    color=color, alpha=0.7, linewidth=1.5, label=f"Drone {drone_id} planned")
        except FileNotFoundError:
            print(f"WARNING: waypoints_drone{drone_id}.csv not found, skipping planned path.")

        try:
            acx, acy = load_actual(drone_id, HOME_LAT, HOME_LON)
            print(f"Drone {drone_id} actual points : {len(acx)}  "
                  f"x range: {min(acx):.2f} -> {max(acx):.2f} m  "
                  f"y range: {min(acy):.2f} -> {max(acy):.2f} m")
            ax.plot(acx, acy, linestyle="-", color=color, alpha=0.9,
                    linewidth=1.3, label=f"Drone {drone_id} actual")
        except FileNotFoundError:
            print(f"WARNING: actual_path_drone{drone_id}.csv not found, skipping actual path.")

    ax.axis("equal")
    ax.grid(True, alpha=0.3)
    ax.set_xlabel("East (m)")
    ax.set_ylabel("North (m)")
    ax.set_title("3-Drone Boustrophedon Coverage - Planned vs Actual")
    ax.legend(fontsize=8)

    out = os.path.join(WAYPOINT_DIR, "combined_planned_vs_actual.png")
    fig.savefig(out, dpi=150, bbox_inches="tight")
    print(f"\nSaved -> {out}")


if __name__ == "__main__":
    main()
