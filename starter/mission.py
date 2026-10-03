"""Shared flight sequences and mission logic for SITL drone."""

from __future__ import annotations

import logging
from collections.abc import Iterator
from contextlib import contextmanager
from typing import Callable

from starter.drone import Drone, DroneError
from starter.geo import square_corners

logger = logging.getLogger(__name__)


def takeoff_sequence(drone: Drone, altitude_m: float = 15.0) -> None:
    """Execute complete takeoff sequence: ready check -> GUIDED -> arm -> takeoff."""
    drone.wait_ready_to_arm(timeout=120.0)
    drone.set_mode("GUIDED", timeout=10.0)
    drone.arm(timeout=30.0)
    drone.takeoff(altitude_m=altitude_m, timeout=60.0)


def rtl_and_wait(drone: Drone, timeout: float = 180.0) -> None:
    """Command RTL mode and wait until drone touches down and disarms."""
    if drone.telemetry.mode not in ("RTL", "LAND"):
        drone.set_mode("RTL", timeout=10.0)
    drone.wait_landed_disarmed(timeout=timeout)


def recover_airborne(drone: Drone, timeout: float = 180.0) -> None:
    """Best-effort RTL while the MAVLink connection is still open."""
    if not drone.telemetry.armed:
        return
    drone.log.warning(
        "Flight interrupted while armed; requesting RTL and waiting for landing..."
    )
    try:
        if drone.telemetry.mode not in ("RTL", "LAND"):
            try:
                drone.set_mode("RTL", timeout=10.0)
            except DroneError as error:
                drone.log.warning("RTL confirmation failed; retrying without ACK: %s", error)
                drone.request_mode_nowait("RTL")
        drone.wait_landed_disarmed(timeout=timeout)
    except DroneError as error:
        drone.log.error("Emergency landing could not be confirmed: %s", error)


@contextmanager
def flight_guard(drone: Drone) -> Iterator[None]:
    """Keep the transport open for best-effort recovery on flight errors."""
    try:
        yield
    except (DroneError, KeyboardInterrupt):
        recover_airborne(drone)
        raise


def fly_square_mission(
    drone: Drone,
    side_m: float = 80.0,
    altitude_m: float = 15.0,
    leg_timeout: float = 90.0,
    on_leg_start: Callable[[int, str, tuple[float, float]], None] | None = None,
) -> list[tuple[float, float]]:
    """Fly an 80m square route A -> B -> C -> D -> A in GUIDED mode.

    Returns the list of 5 coordinates [A, B, C, D, A].
    """
    snap = drone.telemetry
    corner_a = (snap.lat, snap.lon)
    corners = square_corners(corner_a[0], corner_a[1], side_m=side_m)
    corner_names = ["B", "C", "D", "A"]

    drone.log.info("Starting square mission: 4 legs of %.1f m each", side_m)

    for idx in range(1, 5):
        target = corners[idx]
        cname = corner_names[idx - 1]
        drone.log.info("Leg %d/4 started: target corner %s", idx, cname)

        if on_leg_start:
            on_leg_start(idx, cname, target)

        def make_progress_cb(leg_num: int, target_name: str) -> Callable[[float], None]:
            return lambda dist: drone.log.info(
                "Leg %d/4: %.1f m to corner %s", leg_num, dist, target_name
            )

        drone.fly_to(
            lat=target[0],
            lon=target[1],
            alt_m=altitude_m,
            radius_m=2.0,
            timeout=leg_timeout,
            on_progress=make_progress_cb(idx, cname),
        )
        drone.log.info("Reached corner %s (within 2.0 m)", cname)

    drone.log.info("Square flight completed; returned to start corner A.")
    return corners
