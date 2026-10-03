"""Part 1: Arm and take off to 15 m in GUIDED mode."""

from __future__ import annotations

import argparse
import logging
import os
import sys

from starter.drone import Drone, DroneError

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s.%(msecs)03d %(levelname)-7s %(message)s",
    datefmt="%Y-%m-%dT%H:%M:%S",
)
logger = logging.getLogger("part1")


def main() -> int:
    default_conn = os.getenv("MAVLINK_CONNECTION", "tcp:127.0.0.1:5760")
    parser = argparse.ArgumentParser(description="Part 1 — Arm and take off")
    parser.add_argument(
        "--connect",
        default=default_conn,
        help=f"MAVLink connection string (default: {default_conn})",
    )
    args = parser.parse_args()

    drone_inst: Drone | None = None
    try:
        with Drone(connection_string=args.connect, log=logger) as drone:
            drone_inst = drone
            # 1. Wait until drone is ready (EKF and GPS converge)
            drone.wait_ready_to_arm(timeout=120.0)

            # 2. Switch to GUIDED mode and confirm
            drone.set_mode("GUIDED", timeout=10.0)

            # 3. Arm motors and confirm (ack + heartbeat)
            drone.arm(timeout=30.0)

            # 4. Command takeoff to 15 m and wait until reached
            drone.takeoff(altitude_m=15.0, timeout=60.0)

            logger.info("Part 1 completed successfully.")
            return 0

    except DroneError as e:
        logger.error("ERROR: %s", e)
        return 1
    except KeyboardInterrupt:
        logger.error("Interrupted by user (Ctrl+C)")
        if drone_inst is not None:
            try:
                snap = drone_inst.telemetry
                if snap.armed and snap.relative_alt > 1.0:
                    logger.warning("Attempting emergency RTL before exit...")
                    drone_inst.request_mode_nowait("RTL")
            except Exception:
                pass
        return 130


if __name__ == "__main__":
    sys.exit(main())
