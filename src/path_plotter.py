#!/usr/bin/env python3
"""Plot planned local waypoints vs actual GPS track (converted to local ENU)."""

import csv
import math
import os
import matplotlib.pyplot as plt

WAYPOINT_FILE    = os.path.expanduser('~/coverage_map/waypoints.csv')
ACTUAL_PATH_FILE = os.path.expanduser('~/coverage_map/actual_path.csv')
EARTH_RADIUS_M   = 6378137.0

# ========== USE THE REAL HOME FROM mission_executor ==========
HOME_LAT = -35.3632622
HOME_LON = 149.1652375
# =============================================================


def load_planned(path):
    xs, ys = [], []
    with open(path) as f:
        for row in csv.DictReader(f):
            xs.append(float(row['x_local_m']))
            ys.append(float(row['y_local_m']))
    return xs, ys


def load_actual(path, home_lat, home_lon):
    """Convert lat/lon → local East-North metres (flat-earth)."""
    xs, ys = [], []
    with open(path) as f:
        for row in csv.DictReader(f):
            lat = float(row['lat'])
            lon = float(row['lon'])

            d_lat = math.radians(lat - home_lat)
            d_lon = math.radians(lon - home_lon)

            # East (x), North (y)
            x = d_lon * EARTH_RADIUS_M * math.cos(math.radians(home_lat))
            y = d_lat * EARTH_RADIUS_M

            xs.append(x)
            ys.append(y)
    return xs, ys


def main():
    px, py = load_planned(WAYPOINT_FILE)
    ax, ay = load_actual(ACTUAL_PATH_FILE, HOME_LAT, HOME_LON)

    print(f"Planned points : {len(px)}")
    print(f"Actual points  : {len(ax)}")
    print(f"Planned x range: {min(px):.2f} → {max(px):.2f} m")
    print(f"Planned y range: {min(py):.2f} → {max(py):.2f} m")
    print(f"Actual  x range: {min(ax):.2f} → {max(ax):.2f} m")
    print(f"Actual  y range: {min(ay):.2f} → {max(ay):.2f} m")

    plt.figure(figsize=(10, 8))
    plt.plot(px, py, 'b-o', markersize=4, linewidth=1.5, label='Planned')
    plt.plot(ax, ay, 'r-', alpha=0.8, linewidth=1.3, label='Actual')
    plt.axis('equal')
    plt.grid(True, alpha=0.3)
    plt.xlabel('East (m)')
    plt.ylabel('North (m)')
    plt.title('Boustrophedon Coverage – Planned vs Actual')
    plt.legend()

    out = os.path.expanduser('~/coverage_map/planned_vs_actual.png')
    plt.savefig(out, dpi=150, bbox_inches='tight')
    print(f'\nSaved → {out}')
    plt.show()


if __name__ == '__main__':
    main()
