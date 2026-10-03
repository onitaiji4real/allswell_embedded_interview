"""Bonus B-2: Circular Geofence Monitor.

Defines a circular geofence around Home, emits approach warning at 80% radius,
and forces emergency RTL upon boundary breach.
"""

from __future__ import annotations

import argparse
import logging
import os
import sys
from typing import Any

from starter.drone import Drone, DroneError, MissionAborted
from starter.geo import horizontal_distance_m, offset_latlon
from starter.mission import flight_guard, takeoff_sequence

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s.%(msecs)03d %(levelname)-7s %(message)s",
    datefmt="%Y-%m-%dT%H:%M:%S",
)
logger = logging.getLogger("geofence")


def main() -> int:
    default_conn = os.getenv("MAVLINK_CONNECTION", "tcp:127.0.0.1:5760")
    parser = argparse.ArgumentParser(
        description="Bonus B-2 — Geofence monitor with approach warning and breach RTL"
    )
    parser.add_argument(
        "--connect",
        default=default_conn,
        help=f"MAVLink connection string (default: {default_conn})",
    )
    parser.add_argument(
        "--radius",
        type=float,
        default=100.0,
        help="Geofence radius in meters (default: 100.0m)",
    )
    parser.add_argument(
        "--warn-ratio",
        type=float,
        default=0.8,
        help="Ratio of fence radius to trigger approach warning (default: 0.8 = 80%)",
    )
    args = parser.parse_args()

    fence_radius = args.radius
    warn_radius = fence_radius * args.warn_ratio

    try:
        with Drone(connection_string=args.connect, log=logger) as drone, flight_guard(drone):
            home_pos: dict[str, float] = {}
            monitor_active = False
            state: dict[str, Any] = {"warned": False, "breached": False}

            # ------------------------------------------------------------------
            # Geofence Monitor Listener (RX thread - Rule A: Non-blocking)
            # ------------------------------------------------------------------
            def geofence_listener(msg: Any) -> None:
                if not monitor_active or "lat" not in home_pos:
                    return
                if msg.get_type() == "GLOBAL_POSITION_INT":
                    lat = msg.lat / 1e7
                    lon = msg.lon / 1e7
                    dist = horizontal_distance_m(
                        home_pos["lat"], home_pos["lon"], lat, lon
                    )

                    # 1. Approach Warning (80% boundary)
                    if warn_radius <= dist < fence_radius:
                        if not state["warned"]:
                            state["warned"] = True
                            logger.warning(
                                "GEOFENCE WARNING: Approaching boundary (d = %.1f m >= %.1f m warning limit)",
                                dist,
                                warn_radius,
                            )

                    # 2. Fence Breach Enforcement (100% boundary)
                    elif dist >= fence_radius:
                        if drone.abort_and_request_mode_nowait("RTL"):
                            state["breached"] = True
                            logger.critical(
                                "GEOFENCE BREACH: Distance %.1f m >= %.1f m fence limit! Suppressing goto, requesting emergency RTL",
                                dist,
                                fence_radius,
                            )
                            # The abort flag and RTL request share goto's send lock.

            drone.add_listener(geofence_listener)

            # ------------------------------------------------------------------
            # 1. Takeoff to 15 m
            # ------------------------------------------------------------------
            takeoff_sequence(drone, altitude_m=15.0)

            # Use the autopilot's actual Home, which may differ from the takeoff position.
            home_pos["lat"], home_pos["lon"] = drone.get_home_position()
            logger.info(
                "Geofence initialized: Center=(%.6f, %.6f), Radius=%.1f m, Warning=%.1f m",
                home_pos["lat"],
                home_pos["lon"],
                fence_radius,
                warn_radius,
            )

            # Compute an outbound test waypoint 130m North (intentionally crossing 100m fence)
            test_lat, test_lon = offset_latlon(
                home_pos["lat"], home_pos["lon"], north_m=130.0, east_m=0.0
            )

            # ------------------------------------------------------------------
            # 2. Fly toward outbound waypoint to trigger fence enforcement
            # ------------------------------------------------------------------
            monitor_active = True
            logger.info("Flying outward toward test waypoint (130m North) to test fence breach...")

            outbound_completed = False
            try:
                def on_outbound_progress(dist_to_target: float) -> None:
                    cur = drone.telemetry
                    from_home = horizontal_distance_m(
                        home_pos["lat"], home_pos["lon"], cur.lat, cur.lon
                    )
                    logger.info(
                        "Outbound progress: %.1f m from Home (%.1f m to target)",
                        from_home,
                        dist_to_target,
                    )

                drone.fly_to(
                    lat=test_lat,
                    lon=test_lon,
                    alt_m=15.0,
                    radius_m=2.0,
                    timeout=60.0,
                    on_progress=on_outbound_progress,
                )
                outbound_completed = True
            except MissionAborted:
                logger.info(
                    "Geofence breach abort intercepted by main thread; handling emergency return..."
                )

            # If the drone reached the outbound target without triggering a breach, test failed
            if outbound_completed:
                raise DroneError("Drone reached outbound target without geofence breach triggering")

            if not state["breached"]:
                raise DroneError("Abort occurred but geofence breach was not recorded")

            # ------------------------------------------------------------------
            # 3. Confirm RTL & Wait for Landing and Disarm (abortable=False)
            # ------------------------------------------------------------------
            cur_mode = drone.telemetry.mode
            if cur_mode in ("RTL", "LAND"):
                logger.info("Mode -> %s (confirmed)", cur_mode)
            else:
                drone.set_mode("RTL", timeout=10.0)

            drone.wait_landed_disarmed(timeout=180.0)
            logger.info("Landed and disarmed. Geofence containment mission successful.")
            return 0

    except DroneError as e:
        logger.error("ERROR: %s", e)
        return 1
    except KeyboardInterrupt:
        logger.error("Interrupted by user (Ctrl+C)")
        return 130


if __name__ == "__main__":
    sys.exit(main())
