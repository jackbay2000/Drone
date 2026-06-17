"""
Waypoint file loader.

The Waypoint dataclass is used by the plotter (wp.x, wp.y, wp.z, wp.keep_heading).
load_waypoints() parses a simple CSV into a list of Waypoints.
"""

import csv
import os
from dataclasses import dataclass
from typing import List

_WAYPOINTS_DIR = os.path.join(os.path.dirname(__file__), '..', 'inputs', 'waypoints')
_DEFAULT_FILE  = os.path.join(_WAYPOINTS_DIR, 'waypoints.csv')


@dataclass
class Waypoint:
    x: float
    y: float
    z: float
    keep_heading: bool  # True=lock yaw to 0, False=face next waypoint


def load_waypoints(filepath: str = None, verbose: bool = True) -> List[Waypoint]:
    """
    Load waypoints from a CSV file.

    Format (one waypoint per line, # lines are comments):
      x_m, y_m, z_m, keep_heading
      1.0, 0.0, 1.5, True
      2.0, 1.0, 1.5, False
    """
    path = filepath or _DEFAULT_FILE
    if not os.path.exists(path):
        raise FileNotFoundError(
            f"Waypoint file not found: {path}\n"
            f"Create one in inputs/waypoints/ or pass a path to --waypoints."
        )

    waypoints = []
    with open(path, newline='') as f:
        for raw_line in f:
            line = raw_line.split('#')[0].strip()
            if not line:
                continue
            parts = [p.strip() for p in line.split(',')]
            if len(parts) < 4:
                continue
            try:
                x  = float(parts[0])
                y  = float(parts[1])
                z  = float(parts[2])
                kh = parts[3].lower() in ('true', '1', 'yes')
                waypoints.append(Waypoint(x, y, z, kh))
            except ValueError:
                continue

    if not waypoints:
        raise ValueError(f"No valid waypoints found in {path}")

    if verbose:
        print(f"[waypoints] Loaded {len(waypoints)} waypoints from {os.path.basename(path)}")
        for i, wp in enumerate(waypoints):
            mode = "heading=0°" if wp.keep_heading else "face-next"
            print(f"  WP{i}: ({wp.x:.1f}, {wp.y:.1f}, {wp.z:.1f})  {mode}")

    return waypoints
