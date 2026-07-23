"""Skeleton for your drone controller.

This handles the boring plumbing (connecting, heartbeats, telemetry
streams). The flight logic is yours. Restructure freely — this is a
starting point, not a constraint.
"""

from __future__ import annotations

from pymavlink import mavutil


class DroneError(Exception):
    """Raised when the drone rejects a command or a step times out."""


class Drone:
    def __init__(self, connection_string: str = "tcp:127.0.0.1:5760") -> None:
        self.conn = mavutil.mavlink_connection(connection_string)
        if self.conn.wait_heartbeat(timeout=30) is None:
            raise DroneError("no heartbeat — is SITL running?")
        self.conn.mav.request_data_stream_send(
            self.conn.target_system,
            self.conn.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL,
            4,
            1,
        )

    # ------------------------------------------------------------------
    # TODO: implement the methods below (and add whatever else you need).
    # Hints:
    #   - self.conn.mode_mapping() maps mode names ("GUIDED") to mode IDs.
    #   - Commands go out as COMMAND_LONG; the autopilot replies with
    #     COMMAND_ACK carrying a result code. An ack is not the same thing
    #     as the action having completed — confirm with telemetry too.
    #   - self.conn.recv_match(type=..., blocking=True, timeout=...) is
    #     your friend for waiting on specific messages.
    # ------------------------------------------------------------------

    def set_mode(self, mode_name: str, timeout: float = 10.0) -> None:
        """Switch flight mode and confirm the switch happened."""
        raise NotImplementedError

    def arm(self, timeout: float = 30.0) -> None:
        """Arm motors. Confirm via ack and telemetry."""
        raise NotImplementedError

    def takeoff(self, altitude_m: float, timeout: float = 60.0) -> None:
        """Take off and block until within 0.5 m of the target altitude."""
        raise NotImplementedError

    def goto(self, lat: float, lon: float, alt_m: float) -> None:
        """Command the drone to fly to a global position (GUIDED mode)."""
        raise NotImplementedError
