"""Part 2: Fly an 80m square pattern at 15m altitude, then RTL and land."""

from __future__ import annotations

import argparse
import logging
import os
import sys

from starter.drone import Drone, DroneError
from starter.mission import flight_guard, fly_square_mission, rtl_and_wait, takeoff_sequence

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s.%(msecs)03d %(levelname)-7s %(message)s",
    datefmt="%Y-%m-%dT%H:%M:%S",
)
logger = logging.getLogger("part2")


def main() -> int:
    default_conn = os.getenv("MAVLINK_CONNECTION", "tcp:127.0.0.1:5760")
    parser = argparse.ArgumentParser(description="Part 2 — Fly a square pattern and RTL")
    parser.add_argument(
        "--connect",
        default=default_conn,
        help=f"MAVLink connection string (default: {default_conn})",
    )
    args = parser.parse_args()

    try:
        with Drone(connection_string=args.connect, log=logger) as drone, flight_guard(drone):
            # 1. Takeoff to 15 m
            takeoff_sequence(drone, altitude_m=15.0)

            # 2. Fly square pattern (80 m per side, returning to A)
            fly_square_mission(drone, side_m=80.0, altitude_m=15.0)

            # 3. Command RTL and wait until landed and disarmed
            logger.info("Square mission complete. Commanding RTL...")
            rtl_and_wait(drone, timeout=180.0)

            logger.info("Part 2 completed successfully.")
            return 0

    except DroneError as e:
        logger.error("ERROR: %s", e)
        return 1
    except KeyboardInterrupt:
        logger.error("Interrupted by user (Ctrl+C)")
        return 130


if __name__ == "__main__":
    sys.exit(main())
