#!/usr/bin/env python3
"""
gps_utils.py
=============
GPS ↔ local Cartesian (ENU) conversion utilities for the Mercury UGV.

Uses an equirectangular (flat-earth) approximation anchored at a datum
point — the robot's real-world start position, which maps to (0, 0) in
the map frame.

Accurate to a few centimetres over distances up to ~1 km (more than
enough for a competition track). Matches ENU convention (X=East,
Y=North), consistent with navsat_real.yaml (yaw_offset: 0.0).

Usage (import):
    from mission.gps_utils import gps_to_local, local_to_gps
    x, y = gps_to_local(datum_lat, datum_lon, target_lat, target_lon)
"""

import math


def gps_to_local(
    lat0: float, lon0: float,
    lat:  float, lon:  float,
) -> tuple[float, float]:
    """
    Convert a GPS fix (lat, lon) into local ENU (x, y) metres,
    relative to the datum point (lat0, lon0).

    Parameters
    ----------
    lat0, lon0 : float
        Datum — robot's real-world start GPS fix, which maps to
        map-frame (0, 0).
    lat, lon : float
        Target GPS point to convert.

    Returns
    -------
    (x, y) : (float, float)
        x = East  in metres  (positive = east  of datum)
        y = North in metres  (positive = north of datum)
    """
    lat0_rad = math.radians(lat0)

    # Metres per degree at this latitude (Bowring's series)
    m_per_deg_lat = (
        111132.92
        - 559.82 * math.cos(2 * lat0_rad)
        + 1.175  * math.cos(4 * lat0_rad)
    )
    m_per_deg_lon = (
        111412.84 * math.cos(lat0_rad)
        - 93.5    * math.cos(3 * lat0_rad)
    )

    x = (lon - lon0) * m_per_deg_lon   # East
    y = (lat - lat0) * m_per_deg_lat   # North
    return x, y


def local_to_gps(
    lat0: float, lon0: float,
    x:    float, y:    float,
) -> tuple[float, float]:
    """
    Inverse of gps_to_local — converts local ENU (x, y) metres back
    to GPS (lat, lon).

    Parameters
    ----------
    lat0, lon0 : float
        Datum — same origin used in gps_to_local.
    x, y : float
        Local map-frame position in metres (ENU).

    Returns
    -------
    (lat, lon) : (float, float)
    """
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

    lat = lat0 + (y / m_per_deg_lat)
    lon = lon0 + (x / m_per_deg_lon)
    return lat, lon
