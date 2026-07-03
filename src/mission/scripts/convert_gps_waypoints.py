#!/usr/bin/env python3
"""
convert_gps_waypoints.py
=========================
CLI tool: convert real-world GPS waypoints into local map-frame (x, y)
coordinates ready to paste into mission_params.yaml.

Run this ONCE before each competition deployment, after you have the
robot's start-position GPS fix (datum) and the field waypoint GPS coords.

──────────────────────────────────────────────────────────────────────
USAGE
──────────────────────────────────────────────────────────────────────

  python3 convert_gps_waypoints.py \\
      --datum_lat 28.7531 \\
      --datum_lon 77.1177 \\
      --waypoints "WP-1,28.75325,77.11790" \\
                  "WP-2,28.75310,77.11800" \\
                  "WP-3,28.75290,77.11770"

Each --waypoints entry is:  "NAME,latitude,longitude"

──────────────────────────────────────────────────────────────────────
OUTPUT (example)
──────────────────────────────────────────────────────────────────────

  === Converted waypoints ===

  NAME          LAT          LON        x (E) m    y (N) m
  ----          ---          ---        -------    -------
  WP-1      28.75325    77.11790        16.541       1.664
  WP-2      28.75310    77.11800        17.429      -0.001
  WP-3      28.75290    77.11770        14.647      -2.225

  === Paste this block into mission_params.yaml > waypoints > ros__parameters ===

  waypoints:
  - 16.541
  - 1.664
  - 17.429
  - -0.001
  - 14.647
  - -2.225
  waypoint_names:
  - WP-1
  - WP-2
  - WP-3

──────────────────────────────────────────────────────────────────────
HOW TO GET THE DATUM
──────────────────────────────────────────────────────────────────────

1. Place the robot at the competition start line.
2. Echo the GPS topic for a few seconds and note a stable fix:

       ros2 topic echo /gps --once

3. Use the latitude and longitude from that message as
   --datum_lat and --datum_lon.

That datum point will be map-frame (0, 0) because navsat_transform_node
(wait_for_datum: false) uses the first GPS fix as its origin.

──────────────────────────────────────────────────────────────────────
"""

import argparse
import sys
import math


# ── Core math (mirrors mission.gps_utils, self-contained here so this
#    script works without a ROS/colcon install) ─────────────────────

def gps_to_local(
    lat0: float, lon0: float,
    lat:  float, lon:  float,
) -> tuple[float, float]:
    """GPS (lat, lon) → local ENU (x, y) metres relative to datum."""
    lat0_rad = math.radians(lat0)
    m_per_deg_lat = (
        111132.92
        - 559.82 * math.cos(2 * lat0_rad)
        + 1.175  * math.cos(4 * lat0_rad)
    )
    m_per_deg_lon = (
        111412.84 * math.cos(lat0_rad)
        - 93.5    * math.cos(3 * lat0_rad)
    )
    x = (lon - lon0) * m_per_deg_lon
    y = (lat - lat0) * m_per_deg_lat
    return x, y


# ── Argument parsing ───────────────────────────────────────────────

def parse_args():
    p = argparse.ArgumentParser(
        description="Convert GPS waypoints to local map-frame (x, y) for Mercury UGV.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    p.add_argument(
        "--datum_lat", type=float, required=True,
        help="Datum latitude  — robot start GPS fix (maps to map-frame y=0)",
    )
    p.add_argument(
        "--datum_lon", type=float, required=True,
        help="Datum longitude — robot start GPS fix (maps to map-frame x=0)",
    )
    p.add_argument(
        "--waypoints", nargs="+", required=True,
        metavar="NAME,LAT,LON",
        help=(
            'One or more waypoints as "NAME,latitude,longitude". '
            'Example: "WP-1,28.75325,77.11790"'
        ),
    )
    p.add_argument(
        "--precision", type=int, default=3,
        help="Decimal places for output metres (default: 3)",
    )
    return p.parse_args()


# ── Parse a single "NAME,lat,lon" token ───────────────────────────

def parse_waypoint(token: str):
    parts = token.strip().split(",")
    if len(parts) != 3:
        print(
            f"[ERROR] Bad waypoint format: '{token}'\n"
            "        Expected: NAME,latitude,longitude",
            file=sys.stderr,
        )
        sys.exit(1)
    name = parts[0].strip()
    try:
        lat = float(parts[1])
        lon = float(parts[2])
    except ValueError:
        print(
            f"[ERROR] Cannot parse lat/lon in: '{token}'",
            file=sys.stderr,
        )
        sys.exit(1)
    return name, lat, lon


# ── Main ──────────────────────────────────────────────────────────

def main():
    args = parse_args()

    datum_lat = args.datum_lat
    datum_lon = args.datum_lon
    prec      = args.precision

    waypoints = [parse_waypoint(t) for t in args.waypoints]

    # ── Table header ──────────────────────────────────────────────
    col_w = max(len(wp[0]) for wp in waypoints)
    col_w = max(col_w, 6)

    header = (
        f"\n{'NAME':<{col_w}}  {'LAT':>12}  {'LON':>12}"
        f"  {'x (E) m':>12}  {'y (N) m':>12}"
    )
    sep = "-" * len(header)

    print("\n=== Converted waypoints ===\n")
    print(f"Datum: lat={datum_lat}, lon={datum_lon}  →  map-frame (0, 0)")
    print(sep)
    print(header)
    print(sep)

    results = []
    for name, lat, lon in waypoints:
        x, y = gps_to_local(datum_lat, datum_lon, lat, lon)
        results.append((name, lat, lon, x, y))
        print(
            f"{name:<{col_w}}  {lat:>12.7f}  {lon:>12.7f}"
            f"  {x:>12.{prec}f}  {y:>12.{prec}f}"
        )

    print(sep)

    # ── YAML snippet ──────────────────────────────────────────────
    print(
        "\n=== Paste this block into mission_params.yaml"
        " > waypoints > ros__parameters ===\n"
    )

    flat_vals = []
    names     = []
    for name, _, _, x, y in results:
        flat_vals.append(round(x, prec))
        flat_vals.append(round(y, prec))
        names.append(name)

    print("waypoints:")
    for v in flat_vals:
        print(f"- {v}")
    print("waypoint_names:")
    for n in names:
        print(f"- {n}")

    print(
        "\nDone. Copy the block above into mission_params.yaml, "
        "then rebuild & deploy.\n"
    )


if __name__ == "__main__":
    main()
