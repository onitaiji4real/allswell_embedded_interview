"""Drone controller implementation using pymavlink."""

from __future__ import annotations

import collections
import logging
import os
import threading
import time
from dataclasses import dataclass
from typing import Any, Callable

from pymavlink import mavutil
from starter.geo import horizontal_distance_m

logger = logging.getLogger(__name__)

# Map MAV_RESULT numeric codes to readable names for clear error reporting
MAV_RESULT_NAMES: dict[int, str] = {
    0: "MAV_RESULT_ACCEPTED",
    1: "MAV_RESULT_TEMPORARILY_REJECTED",
    2: "MAV_RESULT_DENIED",
    3: "MAV_RESULT_UNSUPPORTED",
    4: "MAV_RESULT_FAILED",
    5: "MAV_RESULT_IN_PROGRESS",
    6: "MAV_RESULT_CANCELLED",
    7: "MAV_RESULT_COMMAND_LONG_ONLY",
    8: "MAV_RESULT_COMMAND_INT_ONLY",
}


class DroneError(Exception):
    """Raised when the drone rejects a command or an operation times out."""


class MissionAborted(DroneError):
    """Raised when safety monitoring triggers mission abort."""


@dataclass(frozen=True)
class TelemetrySnapshot:
    """Immutable snapshot of the drone's current telemetry state."""

    mode: str = "?"
    armed: bool = False
    lat: float = 0.0  # degrees
    lon: float = 0.0  # degrees
    relative_alt: float = 0.0  # meters relative to home
    alt: float = 0.0  # meters MSL
    voltage_battery: float = 0.0  # Volts
    ekf_flags: int = 0
    gps_fix: int = 0
    landed_state: int = 0
    last_heartbeat_time: float = 0.0
    last_statustext: str = ""


class Drone:
    """High-level MAVLink controller for ArduCopter SITL."""

    def __init__(
        self,
        connection_string: str = "",
        heartbeat_timeout: float = 30.0,
        log: logging.Logger | None = None,
        conn: Any | None = None,
        enable_gcs_heartbeat: bool = False,
    ) -> None:
        if not connection_string:
            connection_string = os.getenv("MAVLINK_CONNECTION", "tcp:127.0.0.1:5760")
        self.log = log or logger
        self.connection_string = connection_string

        # Thread synchronization locks
        self._state_lock = threading.Lock()
        self._send_lock = threading.Lock()
        self._ack_lock = threading.Lock()
        self._param_lock = threading.Lock()
        self._listeners_lock = threading.Lock()

        # Telemetry state dictionary (protected by _state_lock)
        self._state: dict[str, Any] = {
            "mode": "?",
            "armed": False,
            "lat": 0.0,
            "lon": 0.0,
            "relative_alt": 0.0,
            "alt": 0.0,
            "voltage_battery": 0.0,
            "ekf_flags": 0,
            "gps_fix": 0,
            "landed_state": 0,
            "last_heartbeat_time": 0.0,
            "last_statustext": "",
        }

        # ACK and parameter response waiters
        self._ack_waiters: dict[int, list[tuple[threading.Event, list[Any]]]] = (
            collections.defaultdict(list)
        )
        self._param_waiters: dict[str, list[tuple[threading.Event, list[Any]]]] = (
            collections.defaultdict(list)
        )

        # Message listeners (e.g. for safety monitoring in Part 3)
        self._listeners: list[Callable[[Any], None]] = []

        # Lifecycle flags
        self._stop_event = threading.Event()
        self.abort_event = threading.Event()
        self._link_broken = False
        self._last_hb_time = 0.0

        # Establish connection or use injected mock
        if conn is not None:
            self.conn = conn
        else:
            self.log.info("Connecting to %s ...", connection_string)
            try:
                self.conn = mavutil.mavlink_connection(connection_string)
            except Exception as e:
                raise DroneError(f"Failed to connect to {connection_string}: {e}") from None

        try:
            hb = self.conn.wait_heartbeat(timeout=heartbeat_timeout)
            if hb is None:
                raise DroneError(f"No heartbeat received within {heartbeat_timeout}s — is SITL running?")
        except Exception as e:
            if isinstance(e, DroneError):
                raise
            raise DroneError(f"Failed to connect to {connection_string}: {e}") from None

        self.target_system = hb.get_srcSystem()
        self.target_component = hb.get_srcComponent()
        self.conn.target_system = self.target_system
        self.conn.target_component = self.target_component
        self._last_hb_time = time.time()
        self.log.info(
            "Connected: system %d component %d",
            self.target_system,
            self.target_component,
        )

        # Request telemetry stream (4 Hz)
        with self._send_lock:
            self.conn.mav.request_data_stream_send(
                self.target_system,
                self.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_ALL,
                4,
                1,
            )

        # Start single background receiver thread
        self._rx_thread = threading.Thread(target=self._rx_loop, name="Drone-RX", daemon=True)
        self._rx_thread.start()

        # Optional 1 Hz GCS heartbeat thread
        self._gcs_hb_thread: threading.Thread | None = None
        if enable_gcs_heartbeat:
            self._gcs_hb_thread = threading.Thread(
                target=self._gcs_heartbeat_loop, name="Drone-GCS-HB", daemon=True
            )
            self._gcs_hb_thread.start()

    def _gcs_heartbeat_loop(self) -> None:
        """Periodic 1 Hz GCS heartbeat sender."""
        while not self._stop_event.is_set():
            try:
                with self._send_lock:
                    self.conn.mav.heartbeat_send(
                        mavutil.mavlink.MAV_TYPE_GCS,
                        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                        0,
                        0,
                        0,
                    )
            except Exception as e:
                self.log.debug("GCS heartbeat failed: %s", e)
            self._stop_event.wait(1.0)

    @property
    def telemetry(self) -> TelemetrySnapshot:
        """Return an immutable snapshot of the current telemetry."""
        with self._state_lock:
            return TelemetrySnapshot(**self._state)

    def add_listener(self, callback: Callable[[Any], None]) -> None:
        """Register a callback invoked by RX thread for every incoming message.

        NOTE: Rule A: Callback must NEVER wait for ACK, PARAM_VALUE, or block.
        """
        with self._listeners_lock:
            self._listeners.append(callback)

    def close(self) -> None:
        """Close connection and terminate background threads."""
        self._stop_event.set()
        if hasattr(self, "_gcs_hb_thread") and self._gcs_hb_thread and self._gcs_hb_thread.is_alive():
            self._gcs_hb_thread.join(timeout=1.0)
        if hasattr(self, "_rx_thread") and self._rx_thread.is_alive():
            self._rx_thread.join(timeout=1.0)
        if hasattr(self, "conn"):
            try:
                self.conn.close()
            except Exception:
                pass

    def __enter__(self) -> Drone:
        return self

    def __exit__(self, exc_type: Any, exc_val: Any, exc_tb: Any) -> None:
        self.close()

    def _rx_loop(self) -> None:
        """Single reader thread calling recv_match and dispatching messages."""
        while not self._stop_event.is_set():
            try:
                msg = self.conn.recv_match(blocking=True, timeout=0.5)
            except Exception as e:
                if not self._stop_event.is_set():
                    self.log.warning("MAVLink receive exception: %s", e)
                    self._link_broken = True
                break

            now = time.time()
            if msg is None:
                if self._last_hb_time > 0 and now - self._last_hb_time > 15.0:
                    self._link_broken = True
                continue

            mtype = msg.get_type()
            if mtype == "BAD_DATA":
                continue

            # Update telemetry state snapshot
            with self._state_lock:
                if mtype == "HEARTBEAT":
                    # Only accept heartbeats from the autopilot (not GCS or other components)
                    if (
                        msg.get_srcSystem() == self.target_system
                        and msg.get_srcComponent() == self.target_component
                        and msg.type != mavutil.mavlink.MAV_TYPE_GCS
                    ):
                        self._last_hb_time = now
                        self._state["last_heartbeat_time"] = now
                        self._state["mode"] = mavutil.mode_string_v10(msg)
                        self._state["armed"] = bool(
                            msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                        )

                elif mtype == "GLOBAL_POSITION_INT":
                    self._state["lat"] = msg.lat / 1e7
                    self._state["lon"] = msg.lon / 1e7
                    self._state["relative_alt"] = msg.relative_alt / 1000.0
                    self._state["alt"] = msg.alt / 1000.0

                elif mtype == "SYS_STATUS":
                    if msg.voltage_battery not in (0, 0xFFFF):
                        self._state["voltage_battery"] = msg.voltage_battery / 1000.0

                elif mtype == "EKF_STATUS_REPORT":
                    self._state["ekf_flags"] = msg.flags

                elif mtype == "GPS_RAW_INT":
                    self._state["gps_fix"] = msg.fix_type

                elif mtype == "EXTENDED_SYS_STATE":
                    self._state["landed_state"] = getattr(msg, "landed_state", 0)

                elif mtype == "STATUSTEXT":
                    text = msg.text
                    if isinstance(text, bytes):
                        text = text.decode("utf-8", errors="ignore")
                    text = text.rstrip("\x00")
                    self._state["last_statustext"] = text

            # Dispatch COMMAND_ACK
            if mtype == "COMMAND_ACK":
                with self._ack_lock:
                    waiters = self._ack_waiters.get(msg.command, [])
                    for ev, holder in waiters:
                        holder.append(msg)
                        ev.set()

            # Dispatch PARAM_VALUE
            elif mtype == "PARAM_VALUE":
                pid = msg.param_id
                if isinstance(pid, bytes):
                    pid = pid.decode("utf-8", errors="ignore")
                pid = pid.rstrip("\x00")
                with self._param_lock:
                    waiters = self._param_waiters.get(pid, [])
                    for ev, holder in waiters:
                        holder.append(msg)
                        ev.set()

            # Call registered message listeners (e.g. battery monitor)
            with self._listeners_lock:
                listeners_copy = list(self._listeners)
            for listener in listeners_copy:
                try:
                    listener(msg)
                except Exception as e:
                    self.log.error("Exception in listener: %s", e)

    def send_command_long(
        self,
        command: int,
        param1: float = 0.0,
        param2: float = 0.0,
        param3: float = 0.0,
        param4: float = 0.0,
        param5: float = 0.0,
        param6: float = 0.0,
        param7: float = 0.0,
        timeout: float = 5.0,
        retries: int = 2,
    ) -> None:
        """Send a COMMAND_LONG and wait for COMMAND_ACK.

        Handles IN_PROGRESS by continuing to wait, and raises DroneError on rejection.
        """
        cmd_meta = mavutil.mavlink.enums.get("MAV_CMD", {}).get(command, None)
        cmd_name = cmd_meta.name if cmd_meta else f"CMD_{command}"

        for attempt in range(retries + 1):
            waiter_ev = threading.Event()
            waiter_holder: list[Any] = []
            with self._ack_lock:
                self._ack_waiters[command].append((waiter_ev, waiter_holder))

            try:
                with self._send_lock:
                    self.conn.mav.command_long_send(
                        self.conn.target_system,
                        self.conn.target_component,
                        command,
                        attempt,
                        param1,
                        param2,
                        param3,
                        param4,
                        param5,
                        param6,
                        param7,
                    )

                deadline = time.time() + timeout
                while time.time() < deadline:
                    if not waiter_holder:
                        remaining = max(0.05, deadline - time.time())
                        if not waiter_ev.wait(remaining):
                            break  # timed out on this wait
                        waiter_ev.clear()

                    if not waiter_holder:
                        continue
                    ack = waiter_holder.pop(0)

                    if ack.result == mavutil.mavlink.MAV_RESULT_IN_PROGRESS:
                        self.log.debug("Command %s IN_PROGRESS, waiting...", cmd_name)
                        continue

                    if ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                        self.log.debug("Command %s ACCEPTED", cmd_name)
                        return

                    res_str = MAV_RESULT_NAMES.get(ack.result, f"RESULT_{ack.result}")
                    last_status = self.telemetry.last_statustext
                    err_msg = f"{cmd_name} rejected: {res_str}"
                    if last_status:
                        err_msg += f" ({last_status})"
                    raise DroneError(err_msg)

                # Timeout on this attempt
                if attempt < retries:
                    self.log.warning(
                        "Command %s attempt %d timed out (no ACK); retrying...",
                        cmd_name,
                        attempt + 1,
                    )
                else:
                    raise DroneError(f"Command {cmd_name} timed out after {timeout}s (no ACK)")

            finally:
                with self._ack_lock:
                    try:
                        self._ack_waiters[command].remove((waiter_ev, waiter_holder))
                    except ValueError:
                        pass

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
            if self._link_broken:
                raise DroneError(f"Connection lost while waiting for {desc}")
            if abortable and self.abort_event.is_set():
                raise MissionAborted(f"Mission aborted while waiting for {desc}")

            if predicate(self.telemetry):
                return
            self._stop_event.wait(poll_interval)

        raise DroneError(f"Timed out after {timeout}s waiting for {desc}")

    def request_mode_nowait(self, mode_name: str) -> None:
        """Send DO_SET_MODE without waiting for ACK or telemetry.

        Safe to invoke from within listener callbacks running on the RX thread (Rule A).
        """
        mode_map = self.conn.mode_mapping()
        if not mode_map:
            mode_map = {"STABILIZE": 0, "AUTO": 3, "GUIDED": 4, "LOITER": 5, "RTL": 6, "LAND": 9}
        if mode_name not in mode_map:
            self.log.error("Unknown flight mode for request_mode_nowait: %s", mode_name)
            return

        custom_mode = mode_map[mode_name]
        with self._send_lock:
            self.conn.mav.command_long_send(
                self.conn.target_system,
                self.conn.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                0,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                custom_mode,
                0,
                0,
                0,
                0,
                0,
            )

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
            if self._link_broken:
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
            self._stop_event.wait(0.5)

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
                self._stop_event.wait(1.5)

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
            if self._link_broken:
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

            self._stop_event.wait(0.2)

        raise DroneError(
            f"Takeoff timed out after {timeout}s (reached {self.telemetry.relative_alt:.1f} m)"
        )

    def goto(self, lat: float, lon: float, alt_m: float) -> None:
        """Send a single position target (relative altitude in meters).

        Checks abort_event inside send_lock to ensure no goto is sent after abort (Rule B-3).
        """
        with self._send_lock:
            if self.abort_event.is_set():
                raise MissionAborted("Mission abort flag is set; goto suppressed")
            self.conn.mav.set_position_target_global_int_send(
                0,  # time_boot_ms
                self.conn.target_system,
                self.conn.target_component,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                0b0000_1111_1111_1000,  # type_mask: position only
                int(lat * 1e7),
                int(lon * 1e7),
                alt_m,
                0,
                0,
                0,  # vx, vy, vz
                0,
                0,
                0,  # afx, afy, afz
                0,
                0,  # yaw, yaw_rate
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
            if self._link_broken:
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

            self._stop_event.wait(0.2)

        dist = horizontal_distance_m(self.telemetry.lat, self.telemetry.lon, lat, lon)
        raise DroneError(
            f"Timed out after {timeout}s flying to waypoint (remaining distance: {dist:.1f} m)"
        )

    def wait_landed_disarmed(self, timeout: float = 180.0) -> None:
        """Wait until drone lands and disarms (abortable=False)."""
        self.log.info("Waiting for landing and disarm...")
        start_time = time.time()
        last_log = 0.0
        while time.time() - start_time < timeout:
            if self._link_broken:
                raise DroneError("Connection lost while waiting for landing")
            snap = self.telemetry
            if not snap.armed:
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
            self._stop_event.wait(0.5)

        raise DroneError(f"Timed out after {timeout}s waiting for landing and disarm")

    def set_param(
        self,
        name: str,
        value: float,
        timeout: float = 5.0,
        retries: int = 3,
    ) -> float:
        """Set a simulator parameter and confirm via PARAM_VALUE.

        Not aborted by abort_event (Rule B-2).
        """
        param_bytes = name.encode("ascii")
        for attempt in range(retries):
            waiter_ev = threading.Event()
            waiter_holder: list[Any] = []
            with self._param_lock:
                self._param_waiters[name].append((waiter_ev, waiter_holder))

            try:
                with self._send_lock:
                    self.conn.mav.param_set_send(
                        self.conn.target_system,
                        self.conn.target_component,
                        param_bytes,
                        value,
                        mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
                    )
                if waiter_ev.wait(timeout):
                    msg = waiter_holder[0]
                    val = msg.param_value
                    if abs(val - value) < 0.01:
                        return val
                    self.log.warning(
                        "Param %s value mismatch: expected %f, got %f", name, value, val
                    )
                else:
                    self.log.warning("Param %s set attempt %d timed out", name, attempt + 1)
            finally:
                with self._param_lock:
                    try:
                        self._param_waiters[name].remove((waiter_ev, waiter_holder))
                    except ValueError:
                        pass

        raise DroneError(f"Failed to set parameter {name} to {value} after {retries} attempts")
