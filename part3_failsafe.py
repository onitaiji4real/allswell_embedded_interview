"""Part 3: Battery failsafe monitoring with simulated low voltage fault injection."""

from __future__ import annotations

import argparse
import logging
import math
import os
import sys
import threading
from typing import Any

from starter.drone import Drone, DroneError, MissionAborted
from starter.mission import flight_guard, fly_square_mission, takeoff_sequence

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s.%(msecs)03d %(levelname)-7s %(message)s",
    datefmt="%Y-%m-%dT%H:%M:%S",
)
logger = logging.getLogger("part3")


def main() -> int:
    default_conn = os.getenv("MAVLINK_CONNECTION", "tcp:127.0.0.1:5760")
    parser = argparse.ArgumentParser(description="Part 3 — Battery failsafe monitor")
    parser.add_argument(
        "--connect",
        default=default_conn,
        help=f"MAVLink connection string (default: {default_conn})",
    )
    parser.add_argument(
        "--fault-delay",
        type=float,
        default=20.0,
        help="Seconds after square mission starts to inject low voltage fault (default: 20.0s)",
    )
    args = parser.parse_args()
    if not math.isfinite(args.fault_delay) or args.fault_delay < 0:
        parser.error("--fault-delay must be a finite non-negative number")

    try:
        with Drone(connection_string=args.connect, log=logger) as drone, flight_guard(drone):
            leg_info = {"current": 0}
            fault_status: dict[str, Any] = {"confirmed": False, "val": 0.0, "error": None}
            abort_info = {"battery": False}
            fault_stop = threading.Event()
            monitor_active = False

            # ------------------------------------------------------------------
            # Safety Monitor (Listener on RX thread - Rule A: NON-BLOCKING ONLY)
            # ------------------------------------------------------------------
            def battery_monitor_listener(msg: Any) -> None:
                if not monitor_active:
                    return
                if msg.get_type() == "SYS_STATUS":
                    v_raw = getattr(msg, "voltage_battery", 0xFFFF)
                    if v_raw != 0xFFFF:
                        v = v_raw / 1000.0
                        if v < 11.0:
                            if drone.abort_and_request_mode_nowait("RTL"):
                                abort_info["battery"] = True
                                current_leg = leg_info["current"]
                                leg_desc = (
                                    f"during leg {current_leg}/4"
                                    if current_leg > 0
                                    else "during mission"
                                )
                                logger.warning(
                                    "ABORT: battery %.2f V < 11.00 V %s; goto stopped, RTL requested",
                                    v,
                                    leg_desc,
                                )
                                # The abort flag and RTL request were ordered with goto under one lock.

            drone.add_listener(battery_monitor_listener)

            # ------------------------------------------------------------------
            # Fault Injection Thread (~20s after square starts - Rule B-2)
            # ------------------------------------------------------------------
            def inject_fault() -> None:
                if fault_stop.wait(args.fault_delay):
                    return
                logger.info("Fault injection: setting SIM_BATT_VOLTAGE=10.5")
                try:
                    confirmed_val = drone.set_param(
                        "SIM_BATT_VOLTAGE",
                        10.5,
                        timeout=5.0,
                        retries=3,
                    )
                    logger.info(
                        "Fault injection: SIM_BATT_VOLTAGE=%.2f confirmed via PARAM_VALUE",
                        confirmed_val,
                    )
                    fault_status["confirmed"] = True
                    fault_status["val"] = confirmed_val
                except Exception as e:
                    logger.error("Fault injection parameter confirmation failed: %s", e)
                    fault_status["error"] = str(e)

            fault_thread = threading.Thread(
                target=inject_fault,
                name="FaultInjector",
                daemon=True,
            )

            def handle_leg_start(idx: int, cname: str, target: tuple[float, float]) -> None:
                nonlocal monitor_active
                leg_info["current"] = idx
                if idx == 1:
                    monitor_active = True
                    if not fault_thread.is_alive():
                        fault_thread.start()

            # ------------------------------------------------------------------
            # 1. Takeoff to 15 m
            # ------------------------------------------------------------------
            takeoff_sequence(drone, altitude_m=15.0)

            # ------------------------------------------------------------------
            # 2. Square Mission with Fault Injection & Failsafe Handling
            # ------------------------------------------------------------------
            mission_error: DroneError | None = None
            mission_completed_normally = False
            try:
                fly_square_mission(
                    drone,
                    side_m=80.0,
                    altitude_m=15.0,
                    on_leg_start=handle_leg_start,
                )
                mission_completed_normally = not drone.abort_event.is_set()
            except MissionAborted:
                logger.info("Mission abort confirmed by main thread; coordinating safe RTL landing...")
            except DroneError as e:
                mission_error = e
                drone.abort_event.set()
                fault_stop.set()
                logger.error("Mission failed: %s; coordinating safe RTL landing...", e)

            # Even a failed safety test must bring an airborne vehicle home first.
            if mission_completed_normally:
                drone.abort_event.set()
                fault_stop.set()
                logger.error("Square mission completed without battery failsafe triggering; returning home")

            # Confirm RTL and wait for landing before evaluating injection success.
            landing_error: DroneError | None = None
            try:
                current_mode = drone.telemetry.mode
                if current_mode in ("RTL", "LAND"):
                    logger.info("Mode -> %s (confirmed)", current_mode)
                else:
                    drone.set_mode("RTL", timeout=10.0)
                drone.wait_landed_disarmed(timeout=180.0)
            except DroneError as e:
                landing_error = e

            if fault_thread.is_alive():
                # Includes the delayed start and all three 5-second PARAM_VALUE attempts.
                fault_thread.join(timeout=args.fault_delay + 17.0)
            if landing_error is not None:
                raise landing_error

            if mission_error is not None:
                logger.error("ERROR: Mission failed after safe landing: %s", mission_error)
                return 1
            if mission_completed_normally:
                logger.error("ERROR: Battery failsafe did not trigger; landed safely")
                return 1
            if fault_thread.is_alive() or not fault_status["confirmed"]:
                logger.error(
                    "ERROR: Fault injection SIM_BATT_VOLTAGE confirmation failed after safe landing: %s",
                    fault_status.get("error") or "timeout",
                )
                return 1
            if not abort_info["battery"]:
                logger.error("ERROR: Mission aborted without a confirmed low-battery trigger")
                return 1

            logger.info("Landed and disarmed. Mission aborted safely.")
            return 0

    except DroneError as e:
        logger.error("ERROR: %s", e)
        return 1
    except KeyboardInterrupt:
        logger.error("Interrupted by user (Ctrl+C)")
        return 130


if __name__ == "__main__":
    sys.exit(main())
