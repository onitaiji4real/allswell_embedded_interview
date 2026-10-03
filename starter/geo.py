"""Geographic calculations for drone navigation (standard library only)."""

from __future__ import annotations

import math

# WGS-84 semi-major axis (meters)
EARTH_RADIUS_M = 6_378_137.0


def offset_latlon(
    lat: float,
    lon: float,
    north_m: float,
    east_m: float,
) -> tuple[float, float]:
    """Compute new (lat, lon) offset from a starting coordinate by (north_m, east_m)."""
    lat_rad = math.radians(lat)
    dlat_rad = north_m / EARTH_RADIUS_M
    dlon_rad = east_m / (EARTH_RADIUS_M * math.cos(lat_rad))

    new_lat = lat + math.degrees(dlat_rad)
    new_lon = lon + math.degrees(dlon_rad)
    return new_lat, new_lon


def horizontal_distance_m(
    lat1: float,
    lon1: float,
    lat2: float,
    lon2: float,
) -> float:
    """Calculate horizontal surface distance in meters between two lat/lon points."""
    lat1_rad = math.radians(lat1)
    lat2_rad = math.radians(lat2)
    dlat = lat2_rad - lat1_rad
    dlon = math.radians(lon2 - lon1)
    mean_lat = (lat1_rad + lat2_rad) / 2.0

    x = dlon * math.cos(mean_lat)
    y = dlat
    return EARTH_RADIUS_M * math.hypot(x, y)


def square_corners(
    lat: float,
    lon: float,
    side_m: float = 80.0,
) -> list[tuple[float, float]]:
    """Generate square waypoints [A, B, C, D, A] starting at (lat, lon).

    Route:
      A: Start point
      B: A + North side_m
      C: B + East side_m
      D: A + East side_m
      A: Back to start
    """
    corner_a = (lat, lon)
    corner_b = offset_latlon(lat, lon, north_m=side_m, east_m=0.0)
    corner_c = offset_latlon(lat, lon, north_m=side_m, east_m=side_m)
    corner_d = offset_latlon(lat, lon, north_m=0.0, east_m=side_m)
    return [corner_a, corner_b, corner_c, corner_d, corner_a]
