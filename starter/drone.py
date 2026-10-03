"""High-level drone flight and navigation controller."""

from __future__ import annotations

import logging
import time
from typing import Any, Callable

from pymavlink import mavutil
from starter.geo import horizontal_distance_m
from starter.link import (
    MAV_RESULT_NAMES,
    DroneError,
    MavlinkLink,
    MissionAborted,
    TelemetrySnapshot,
)

logger = logging.getLogger(__name__)

# Re-export for backward compatibility with existing scripts and tests
__all__ = [
    "Drone",
    "DroneError",
    "MissionAborted",
    "TelemetrySnapshot",
    "MAV_RESULT_NAMES",
]


class Drone:
    """High-level autonomous vehicle controller coordinating flight behaviors."""

    def __init__(
        self,
        connection_string: str = "",
        heartbeat_timeout: float = 30.0,
        log: logging.Logger | None = None,
        conn: Any | None = None,
        enable_gcs_heartbeat: bool = False,
        link: MavlinkLink | None = None,
    ) -> None:
        self.log = log or logger
        if link is not None:
            self.link = link
        else:
            self.link = MavlinkLink(
                connection_string=connection_string,
                heartbeat_timeout=heartbeat_timeout,
                log=self.log,
                conn=conn,
                enable_gcs_heartbeat=enable_gcs_heartbeat,
            )

    @property
    def telemetry(self) -> TelemetrySnapshot:
        """Return an immutable snapshot of current drone telemetry."""
        return self.link.telemetry

    @property
    def conn(self) -> Any:
        """Access the underlying MAVLink connection."""
        return self.link.conn

    @property
    def abort_event(self) -> Any:
        """Access the flight abort coordination event."""
        return self.link.abort_event

    def add_listener(self, callback: Callable[[Any], None]) -> None:
        """Register a callback on the RX communication thread."""
        self.link.add_listener(callback)

    def send_command_long(self, command: int, *args: Any, **kwargs: Any) -> None:
        """Send COMMAND_LONG and await COMMAND_ACK (delegated to link)."""
        self.link.send_command_long(command, *args, **kwargs)

    def request_mode_nowait(self, mode_name: str) -> None:
        """Send non-blocking mode change command (delegated to link)."""
        self.link.request_mode_nowait(mode_name)

    def abort_and_request_mode_nowait(self, mode_name: str = "RTL") -> bool:
        """Stop navigation and request a safety mode as one ordered operation."""
        return self.link.abort_and_request_mode_nowait(mode_name)

    def set_param(self, name: str, value: float, *args: Any, **kwargs: Any) -> float:
        """Set a simulator parameter and verify response (delegated to link)."""
        return self.link.set_param(name, value, *args, **kwargs)

    def goto(self, lat: float, lon: float, alt_m: float) -> None:
        """Send position target to autopilot (delegated to link)."""
        self.link.goto(lat, lon, alt_m)

    def close(self) -> None:
        """Close connection and terminate transport threads."""
        self.link.close()

    def __enter__(self) -> Drone:
        return self

    def __exit__(self, exc_type: Any, exc_val: Any, exc_tb: Any) -> None:
        self.close()

    # ------------------------------------------------------------------
    # High-Level Autopilot Logic & Flight Sequences
    # ------------------------------------------------------------------

    def wait_until(
        self,
        predicate: Callable[[TelemetrySnapshot], bool],
        timeout: float,
        desc: str,
        poll_interval: float = 0.1,
        abortable: bool = False,
    ) -> None:
        """Poll telemetry snapshot until predicate is True or timeout occurs.

        If abortable=True and abort_event is set, raises MissionAborted.
        """
        start_time = time.time()
        while time.time() - start_time < timeout:
            if self.link.is_link_broken:
                raise DroneError(f"Connection lost while waiting for {desc}")
            if abortable and self.abort_event.is_set():
                raise MissionAborted(f"Mission aborted while waiting for {desc}")

            if predicate(self.telemetry):
                return
            time.sleep(poll_interval)

        raise DroneError(f"Timed out after {timeout}s waiting for {desc}")

    def wait_ready_to_arm(self, timeout: float = 120.0) -> None:
        """Wait until EKF and GPS converge so guided takeoff is accepted."""
        self.log.info("Waiting for drone readiness (EKF and GPS convergence)...")
        start_time = time.time()
        last_log_time = 0.0

        required_ekf = (
            mavutil.mavlink.EKF_ATTITUDE
            | mavutil.mavlink.EKF_VELOCITY_HORIZ
            | mavutil.mavlink.EKF_VELOCITY_VERT
            | mavutil.mavlink.EKF_POS_HORIZ_REL
            | mavutil.mavlink.EKF_POS_HORIZ_ABS
            | mavutil.mavlink.EKF_POS_VERT_ABS
        )
        forbidden_ekf = (
            mavutil.mavlink.EKF_CONST_POS_MODE
            | mavutil.mavlink.EKF_UNINITIALIZED
            | mavutil.mavlink.EKF_GPS_GLITCHING
        )

        while time.time() - start_time < timeout:
            if self.link.is_link_broken:
                raise DroneError("Connection lost while waiting for readiness")

            snap = self.telemetry
            ekf = snap.ekf_flags
            gps_fix = snap.gps_fix

            ekf_ready = (ekf & required_ekf) == required_ekf and (ekf & forbidden_ekf) == 0
            gps_ready = gps_fix >= 3

            if ekf_ready and gps_ready:
                self.log.info(
                    "EKF ready (flags=0x%04x), GPS fix %dD",
                    ekf,
                    gps_fix,
                )
                return

            now = time.time()
            if now - last_log_time >= 5.0:
                last_log_time = now
                self.log.info(
                    "Readiness status: EKF=0x%04x (req: 0x%04x), GPS fix=%d (req: >=3)",
                    ekf,
                    required_ekf,
                    gps_fix,
                )
            time.sleep(0.5)

        snap = self.telemetry
        raise DroneError(
            f"Timed out after {timeout}s waiting for readiness (EKF=0x{snap.ekf_flags:04x}, GPS fix={snap.gps_fix})"
        )

    def set_mode(self, mode_name: str, timeout: float = 10.0) -> None:
        """Switch flight mode with dual confirmation (ACK and HEARTBEAT)."""
        mode_map = self.conn.mode_mapping()
        if not mode_map:
            mode_map = {"STABILIZE": 0, "AUTO": 3, "GUIDED": 4, "LOITER": 5, "RTL": 6, "LAND": 9}
        if mode_name not in mode_map:
            raise DroneError(f"Unknown flight mode: {mode_name}")
        custom_mode = mode_map[mode_name]

        self.log.info("Switching mode to %s ...", mode_name)
        self.send_command_long(
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            param1=mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            param2=custom_mode,
            timeout=timeout / 2.0,
        )
        self.wait_until(
            lambda snap: snap.mode == mode_name,
            timeout=timeout / 2.0,
            desc=f"mode switch to {mode_name}",
            abortable=False,
        )
        self.log.info("Mode -> %s (confirmed)", mode_name)

    def arm(self, timeout: float = 30.0) -> None:
        """Arm motors with dual confirmation (ACK and HEARTBEAT), retrying on rejection."""
        self.log.info("Arming motors ...")
        start_time = time.time()
        last_err = ""

        while time.time() - start_time < timeout:
            try:
                self.send_command_long(
                    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                    param1=1.0,  # 1 = Arm
                    timeout=3.0,
                    retries=0,
                )
                rem = max(1.0, timeout - (time.time() - start_time))
                self.wait_until(
                    lambda snap: snap.armed,
                    timeout=min(5.0, rem),
                    desc="armed telemetry confirmation",
                    abortable=False,
                )
                self.log.info("Armed (ack + heartbeat)")
                return
            except DroneError as e:
                last_err = str(e)
                self.log.warning("Arm attempt: %s; retrying...", e)
                time.sleep(1.5)

        raise DroneError(f"Failed to arm within {timeout}s: {last_err}")

    def takeoff(
        self,
        altitude_m: float,
        timeout: float = 60.0,
        on_progress: Callable[[float], None] | None = None,
    ) -> None:
        """Command takeoff to target altitude (m above home) and wait until reached."""
        self.log.info("Commanding takeoff to %.1f m ...", altitude_m)
        if not self.telemetry.armed:
            raise DroneError("Cannot takeoff: drone is disarmed")
        if self.telemetry.mode != "GUIDED":
            raise DroneError(f"Cannot takeoff: drone in {self.telemetry.mode}, expected GUIDED")

        self.send_command_long(
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            param7=altitude_m,
            timeout=5.0,
        )

        start_time = time.time()
        last_log_time = 0.0
        while time.time() - start_time < timeout:
            if self.link.is_link_broken:
                raise DroneError("Connection lost during takeoff climb")
            snap = self.telemetry

            if not snap.armed:
                raise DroneError("Drone disarmed unexpectedly during takeoff climb")
            if snap.mode != "GUIDED":
                raise DroneError(f"Mode unexpectedly changed to {snap.mode} during takeoff climb")

            rel_alt = snap.relative_alt
            now = time.time()
            if now - last_log_time >= 1.0:
                last_log_time = now
                self.log.info("Climbing: altitude = %.1f m (target: %.1f m)", rel_alt, altitude_m)
                if on_progress:
                    on_progress(rel_alt)

            if abs(rel_alt - altitude_m) <= 0.5:
                self.log.info("REACHED %.1f m", rel_alt)
                return

            time.sleep(0.2)

        raise DroneError(
            f"Takeoff timed out after {timeout}s (reached {self.telemetry.relative_alt:.1f} m)"
        )

    def fly_to(
        self,
        lat: float,
        lon: float,
        alt_m: float,
        radius_m: float = 2.0,
        timeout: float = 90.0,
        on_progress: Callable[[float], None] | None = None,
    ) -> None:
        """Fly toward a waypoint until within radius_m horizontally.

        Periodically sends goto commands (~1 Hz), reports distance, and aborts
        immediately if abort_event is set (Rule B-1).
        """
        start_time = time.time()
        last_send_time = 0.0

        while time.time() - start_time < timeout:
            if self.link.is_link_broken:
                raise DroneError("Connection lost while flying to waypoint")
            if self.abort_event.is_set():
                raise MissionAborted("Mission abort flag is set; fly_to aborted")

            snap = self.telemetry
            if not snap.armed:
                raise DroneError("Drone disarmed unexpectedly during flight")
            if snap.mode != "GUIDED":
                raise DroneError(f"Mode unexpectedly changed to {snap.mode} during flight")

            dist = horizontal_distance_m(snap.lat, snap.lon, lat, lon)
            if dist <= radius_m:
                return

            now = time.time()
            if now - last_send_time >= 1.0:
                last_send_time = now
                self.goto(lat, lon, alt_m)
                if on_progress:
                    on_progress(dist)

            time.sleep(0.2)

        dist = horizontal_distance_m(self.telemetry.lat, self.telemetry.lon, lat, lon)
        raise DroneError(
            f"Timed out after {timeout}s flying to waypoint (remaining distance: {dist:.1f} m)"
        )

    def wait_landed_disarmed(self, timeout: float = 180.0) -> None:
        """Wait for disarm and a recent ground-level position (abortable=False)."""
        self.log.info("Waiting for landing and disarm...")
        start_time = time.time()
        last_log = 0.0
        while time.time() - start_time < timeout:
            if self.link.is_link_broken:
                raise DroneError("Connection lost while waiting for landing")
            snap = self.telemetry
            position_is_fresh = (
                snap.last_position_time > 0
                and time.time() - snap.last_position_time <= 2.0
            )
            if not snap.armed and position_is_fresh and abs(snap.relative_alt) <= 0.5:
                self.log.info("Landed and disarmed. Mission complete.")
                return
            now = time.time()
            if now - last_log >= 5.0:
                last_log = now
                self.log.info(
                    "Landing progress: mode=%s, armed=%s, alt=%.1f m",
                    snap.mode,
                    snap.armed,
                    snap.relative_alt,
                )
            time.sleep(0.5)

        raise DroneError(f"Timed out after {timeout}s waiting for landing and disarm")
