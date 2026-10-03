"""Unit tests for geographic calculations."""

import math
import pytest

from starter.geo import EARTH_RADIUS_M, horizontal_distance_m, offset_latlon, square_corners

# Canberra CMAC test site coordinates from sitl/Dockerfile
HOME_LAT = -35.363261
HOME_LON = 149.165230


def test_offset_north() -> None:
    lat2, lon2 = offset_latlon(HOME_LAT, HOME_LON, north_m=80.0, east_m=0.0)
    dist = horizontal_distance_m(HOME_LAT, HOME_LON, lat2, lon2)
    assert pytest.approx(80.0, abs=0.05) == dist
    assert lon2 == pytest.approx(HOME_LON, abs=1e-7)
    assert lat2 > HOME_LAT  # North is greater latitude


def test_offset_east() -> None:
    lat2, lon2 = offset_latlon(HOME_LAT, HOME_LON, north_m=0.0, east_m=80.0)
    dist = horizontal_distance_m(HOME_LAT, HOME_LON, lat2, lon2)
    assert pytest.approx(80.0, abs=0.05) == dist
    assert lat2 == pytest.approx(HOME_LAT, abs=1e-7)
    assert lon2 > HOME_LON  # East is greater longitude


def test_distance_symmetry() -> None:
    p1 = (HOME_LAT, HOME_LON)
    p2 = offset_latlon(HOME_LAT, HOME_LON, north_m=50.0, east_m=-30.0)
    d1 = horizontal_distance_m(p1[0], p1[1], p2[0], p2[1])
    d2 = horizontal_distance_m(p2[0], p2[1], p1[0], p1[1])
    assert pytest.approx(d1, abs=1e-6) == d2


def test_square_corners_sides_and_diagonals() -> None:
    corners = square_corners(HOME_LAT, HOME_LON, side_m=80.0)
    assert len(corners) == 5  # A, B, C, D, A

    # A -> B (North 80m)
    d_ab = horizontal_distance_m(corners[0][0], corners[0][1], corners[1][0], corners[1][1])
    assert pytest.approx(80.0, abs=0.05) == d_ab

    # B -> C (East 80m)
    d_bc = horizontal_distance_m(corners[1][0], corners[1][1], corners[2][0], corners[2][1])
    assert pytest.approx(80.0, abs=0.05) == d_bc

    # C -> D (South 80m)
    d_cd = horizontal_distance_m(corners[2][0], corners[2][1], corners[3][0], corners[3][1])
    assert pytest.approx(80.0, abs=0.05) == d_cd

    # D -> A (West 80m)
    d_da = horizontal_distance_m(corners[3][0], corners[3][1], corners[4][0], corners[4][1])
    assert pytest.approx(80.0, abs=0.05) == d_da

    # Start and end must match exactly
    assert corners[0] == corners[4]

    # Diagonals (80 * sqrt(2) ≈ 113.137 m)
    expected_diag = 80.0 * math.sqrt(2.0)
    d_ac = horizontal_distance_m(corners[0][0], corners[0][1], corners[2][0], corners[2][1])
    d_bd = horizontal_distance_m(corners[1][0], corners[1][1], corners[3][0], corners[3][1])
    assert pytest.approx(expected_diag, abs=0.1) == d_ac
    assert pytest.approx(expected_diag, abs=0.1) == d_bd
