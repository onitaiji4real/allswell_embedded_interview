"""Unit tests for Drone logic using a mock MAVLink connection (no SITL required)."""

from __future__ import annotations

import queue
import threading
import time
from dataclasses import FrozenInstanceError
from typing import Any
import pytest
from pymavlink import mavutil

from starter.drone import Drone, DroneError, MissionAborted
from starter.geo import horizontal_distance_m, offset_latlon


class MockMsg:
    """Generic mock message simulating pymavlink message objects."""

    def __init__(self, mtype: str, **kwargs: Any) -> None:
        self._type = mtype
        for k, v in kwargs.items():
            setattr(self, k, v)

    def get_type(self) -> str:
        return self._type

    def get_srcSystem(self) -> int:
        return getattr(self, "_src_sys", 1)

    def get_srcComponent(self) -> int:
        return getattr(self, "_src_comp", 1)


class MockMav:
    def __init__(self) -> None:
        self.sent_commands: list[dict[str, Any]] = []
        self.sent_params: list[dict[str, Any]] = []
        self.sent_positions: list[dict[str, Any]] = []

    def request_data_stream_send(self, *args: Any, **kwargs: Any) -> None:
        pass

    def command_long_send(
        self,
        target_system: int,
        target_component: int,
        command: int,
        confirmation: int,
        p1: float,
        p2: float,
        p3: float,
        p4: float,
        p5: float,
        p6: float,
        p7: float,
    ) -> None:
        self.sent_commands.append(
            {
                "command": command,
                "confirmation": confirmation,
                "params": [p1, p2, p3, p4, p5, p6, p7],
            }
        )

    def param_set_send(
        self,
        target_system: int,
        target_component: int,
        param_id: bytes,
        param_value: float,
        param_type: int,
    ) -> None:
        self.sent_params.append({"param_id": param_id, "param_value": param_value})

    def set_position_target_global_int_send(self, *args: Any, **kwargs: Any) -> None:
        self.sent_positions.append({"args": args, "kwargs": kwargs})

    def heartbeat_send(self, *args: Any, **kwargs: Any) -> None:
        pass


class MockConnection:
    """Mock pymavlink connection object fed by an in-memory queue."""

    def __init__(self) -> None:
        self.target_system = 1
        self.target_component = 1
        self.mav = MockMav()
        self.rx_queue: queue.Queue[Any] = queue.Queue()
        self.closed = False

    def wait_heartbeat(self, timeout: float = 1.0) -> MockMsg:
        return MockMsg(
            "HEARTBEAT",
            type=mavutil.mavlink.MAV_TYPE_QUADROTOR,
            autopilot=mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
            base_mode=0,
            custom_mode=0,
            _src_sys=1,
            _src_comp=1,
        )

    def mode_mapping(self) -> dict[str, int]:
        return {"STABILIZE": 0, "GUIDED": 4, "RTL": 6}

    def recv_match(self, blocking: bool = True, timeout: float = 0.5) -> Any:
        if self.closed:
            return None
        try:
            return self.rx_queue.get(block=blocking, timeout=timeout)
        except queue.Empty:
            return None

    def close(self) -> None:
        self.closed = True


@pytest.fixture
def mock_drone() -> tuple[Drone, MockConnection]:
    conn = MockConnection()
    drone = Drone(conn=conn, enable_gcs_heartbeat=False)
    yield drone, conn
    drone.close()


def test_command_ack_accepted(mock_drone: tuple[Drone, MockConnection]) -> None:
    drone, conn = mock_drone

    # Enqueue accepted ACK
    ack = MockMsg(
        "COMMAND_ACK",
        command=mavutil.mavlink.MAV_CMD_DO_SET_MODE,
        result=mavutil.mavlink.MAV_RESULT_ACCEPTED,
    )
    conn.rx_queue.put(ack)

    # Should succeed without error
    drone.send_command_long(mavutil.mavlink.MAV_CMD_DO_SET_MODE, timeout=1.0)
    assert len(conn.mav.sent_commands) == 1
    assert conn.mav.sent_commands[0]["command"] == mavutil.mavlink.MAV_CMD_DO_SET_MODE


def test_command_ack_rejected(mock_drone: tuple[Drone, MockConnection]) -> None:
    drone, conn = mock_drone

    # Enqueue rejected ACK
    ack = MockMsg(
        "COMMAND_ACK",
        command=mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        result=mavutil.mavlink.MAV_RESULT_TEMPORARILY_REJECTED,
    )
    conn.rx_queue.put(ack)

    with pytest.raises(DroneError, match="MAV_RESULT_TEMPORARILY_REJECTED"):
        drone.send_command_long(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, timeout=1.0)


def test_command_ack_in_progress_then_accepted(
    mock_drone: tuple[Drone, MockConnection]
) -> None:
    drone, conn = mock_drone

    # Enqueue IN_PROGRESS followed by ACCEPTED
    ack1 = MockMsg(
        "COMMAND_ACK",
        command=mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        result=mavutil.mavlink.MAV_RESULT_IN_PROGRESS,
    )
    ack2 = MockMsg(
        "COMMAND_ACK",
        command=mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        result=mavutil.mavlink.MAV_RESULT_ACCEPTED,
    )
    conn.rx_queue.put(ack1)
    conn.rx_queue.put(ack2)

    drone.send_command_long(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, timeout=2.0)
    assert len(conn.mav.sent_commands) == 1


def test_command_ack_timeout_with_retries(mock_drone: tuple[Drone, MockConnection]) -> None:
    drone, conn = mock_drone

    # Send command with no ACK in queue
    with pytest.raises(DroneError, match="timed out"):
        drone.send_command_long(mavutil.mavlink.MAV_CMD_DO_SET_MODE, timeout=0.2, retries=1)

    # Should have sent initial attempt (0) + 1 retry (1)
    assert len(conn.mav.sent_commands) == 2
    assert conn.mav.sent_commands[0]["confirmation"] == 0
    assert conn.mav.sent_commands[1]["confirmation"] == 1


def test_param_set_and_confirm(mock_drone: tuple[Drone, MockConnection]) -> None:
    drone, conn = mock_drone

    # Enqueue PARAM_VALUE response
    pv = MockMsg(
        "PARAM_VALUE",
        param_id=b"SIM_BATT_VOLTAGE\x00\x00",
        param_value=10.5,
    )
    conn.rx_queue.put(pv)

    val = drone.set_param("SIM_BATT_VOLTAGE", 10.5, timeout=1.0)
    assert val == 10.5
    assert len(conn.mav.sent_params) == 1
    assert conn.mav.sent_params[0]["param_id"] == b"SIM_BATT_VOLTAGE"


def test_telemetry_snapshot_updates_and_immutable(
    mock_drone: tuple[Drone, MockConnection]
) -> None:
    drone, conn = mock_drone

    # Push telemetry updates
    pos = MockMsg(
        "GLOBAL_POSITION_INT",
        lat=-353632610,
        lon=1491652300,
        relative_alt=15000,
        alt=599000,
    )
    sys_status = MockMsg("SYS_STATUS", voltage_battery=12600)
    conn.rx_queue.put(pos)
    conn.rx_queue.put(sys_status)

    time.sleep(0.1)  # Allow RX thread to process

    snap = drone.telemetry
    assert snap.lat == pytest.approx(-35.363261)
    assert snap.lon == pytest.approx(149.165230)
    assert snap.relative_alt == pytest.approx(15.0)
    assert snap.voltage_battery == pytest.approx(12.6)

    # Snapshot must be frozen/immutable
    with pytest.raises(FrozenInstanceError):
        snap.relative_alt = 20.0  # type: ignore


def test_abort_event_suppresses_goto(mock_drone: tuple[Drone, MockConnection]) -> None:
    drone, conn = mock_drone

    # Setting abort_event immediately causes goto to raise MissionAborted (Rule B-3)
    drone.abort_event.set()
    with pytest.raises(MissionAborted, match="goto suppressed"):
        drone.goto(-35.363, 149.165, 15.0)

    # Verify no position command was sent to MAVLink
    assert len(conn.mav.sent_positions) == 0


def test_abort_waits_for_inflight_goto_before_setting_flag(
    mock_drone: tuple[Drone, MockConnection]
) -> None:
    drone, conn = mock_drone
    send_entered = threading.Event()
    release_send = threading.Event()
    order: list[str] = []
    original_goto = conn.mav.set_position_target_global_int_send
    original_command = conn.mav.command_long_send

    def delayed_goto(*args: Any, **kwargs: Any) -> None:
        send_entered.set()
        assert release_send.wait(2)
        order.append("goto")
        original_goto(*args, **kwargs)

    def record_command(*args: Any, **kwargs: Any) -> None:
        order.append("RTL")
        original_command(*args, **kwargs)

    conn.mav.set_position_target_global_int_send = delayed_goto
    conn.mav.command_long_send = record_command
    goto_worker = threading.Thread(target=drone.goto, args=(-35.363, 149.165, 15.0))
    abort_worker = threading.Thread(target=drone.abort_and_request_mode_nowait)
    goto_worker.start()
    assert send_entered.wait(2)
    abort_worker.start()
    assert not drone.abort_event.is_set()
    release_send.set()
    goto_worker.join(2)
    abort_worker.join(2)

    assert not goto_worker.is_alive() and not abort_worker.is_alive()
    assert order == ["goto", "RTL"]
    assert drone.abort_event.is_set()
    with pytest.raises(MissionAborted):
        drone.goto(-35.363, 149.165, 15.0)
    assert len(conn.mav.sent_positions) == 1


def test_foreign_messages_cannot_update_state_or_confirm_commands(
    mock_drone: tuple[Drone, MockConnection]
) -> None:
    drone, conn = mock_drone
    seen: list[str] = []
    processed = threading.Event()

    def record_message(msg: Any) -> None:
        seen.append(msg.get_type())
        processed.set()

    drone.add_listener(record_message)
    conn.rx_queue.put(MockMsg("SYS_STATUS", voltage_battery=10500, _src_sys=99))
    conn.rx_queue.put(MockMsg("SYS_STATUS", voltage_battery=0, _src_sys=1))
    assert processed.wait(1)
    # The matching zero is valid; the foreign 10.5 V must not reach the listener.
    assert seen == ["SYS_STATUS"]
    assert drone.telemetry.voltage_battery == 0

    def ack_after_send() -> None:
        deadline = time.monotonic() + 2
        while not conn.mav.sent_commands and time.monotonic() < deadline:
            time.sleep(0.01)
        conn.rx_queue.put(
            MockMsg(
                "COMMAND_ACK",
                command=mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                result=mavutil.mavlink.MAV_RESULT_DENIED,
                _src_sys=99,
            )
        )
        conn.rx_queue.put(
            MockMsg(
                "COMMAND_ACK",
                command=mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                result=mavutil.mavlink.MAV_RESULT_ACCEPTED,
                _src_sys=1,
            )
        )

    feeder = threading.Thread(target=ack_after_send)
    feeder.start()
    drone.send_command_long(mavutil.mavlink.MAV_CMD_DO_SET_MODE, timeout=1.0, retries=0)
    feeder.join(2)
    assert not feeder.is_alive()


def test_send_socket_error_becomes_drone_error(
    mock_drone: tuple[Drone, MockConnection]
) -> None:
    drone, conn = mock_drone

    def broken_send(*args: Any, **kwargs: Any) -> None:
        raise OSError("socket closed")

    conn.mav.command_long_send = broken_send
    with pytest.raises(DroneError, match="Failed to send MAV_CMD_DO_SET_MODE: socket closed"):
        drone.send_command_long(mavutil.mavlink.MAV_CMD_DO_SET_MODE, retries=0)


def test_abort_remains_active_when_first_rtl_send_fails(
    mock_drone: tuple[Drone, MockConnection]
) -> None:
    drone, conn = mock_drone

    def broken_send(*args: Any, **kwargs: Any) -> None:
        raise OSError("socket closed")

    conn.mav.command_long_send = broken_send
    assert drone.abort_and_request_mode_nowait("RTL")
    assert drone.abort_event.is_set()
    with pytest.raises(MissionAborted):
        drone.goto(-35.363, 149.165, 15.0)


def test_disarmed_at_altitude_is_not_landed(
    mock_drone: tuple[Drone, MockConnection]
) -> None:
    drone, conn = mock_drone
    conn.rx_queue.put(MockMsg("GLOBAL_POSITION_INT", lat=0, lon=0, relative_alt=15000, alt=0))
    drone.wait_until(lambda snap: snap.relative_alt == 15.0, timeout=1.0, desc="position")
    with pytest.raises(DroneError, match="Timed out"):
        drone.wait_landed_disarmed(timeout=0.1)

    conn.rx_queue.put(MockMsg("GLOBAL_POSITION_INT", lat=0, lon=0, relative_alt=100, alt=0))
    drone.wait_until(lambda snap: snap.relative_alt == 0.1, timeout=1.0, desc="ground position")
    drone.wait_landed_disarmed(timeout=1.0)


def test_geofence_approach_and_breach_detection(
    mock_drone: tuple[Drone, MockConnection]
) -> None:
    drone, conn = mock_drone
    home_lat = -35.363261
    home_lon = 149.165230
    fence_radius = 100.0
    warn_radius = 80.0

    events: list[tuple[str, float]] = []

    def geofence_listener(msg: Any) -> None:
        if msg.get_type() == "GLOBAL_POSITION_INT":
            lat = msg.lat / 1e7
            lon = msg.lon / 1e7
            dist = horizontal_distance_m(home_lat, home_lon, lat, lon)
            if warn_radius <= dist < fence_radius:
                events.append(("WARN", dist))
            elif dist >= fence_radius:
                events.append(("BREACH", dist))
                drone.abort_event.set()
                drone.request_mode_nowait("RTL")

    drone.add_listener(geofence_listener)

    # 1. Safe zone (50m)
    p_safe_lat, p_safe_lon = offset_latlon(home_lat, home_lon, north_m=50.0, east_m=0.0)
    conn.rx_queue.put(
        MockMsg(
            "GLOBAL_POSITION_INT",
            lat=int(p_safe_lat * 1e7),
            lon=int(p_safe_lon * 1e7),
            relative_alt=15000,
            alt=599000,
        )
    )
    time.sleep(0.05)
    assert len(events) == 0
    assert not drone.abort_event.is_set()

    # 2. Warning zone (85m)
    p_warn_lat, p_warn_lon = offset_latlon(home_lat, home_lon, north_m=85.0, east_m=0.0)
    conn.rx_queue.put(
        MockMsg(
            "GLOBAL_POSITION_INT",
            lat=int(p_warn_lat * 1e7),
            lon=int(p_warn_lon * 1e7),
            relative_alt=15000,
            alt=599000,
        )
    )
    time.sleep(0.05)
    assert len(events) == 1
    assert events[0][0] == "WARN"
    assert not drone.abort_event.is_set()

    # 3. Breach zone (105m)
    p_breach_lat, p_breach_lon = offset_latlon(home_lat, home_lon, north_m=105.0, east_m=0.0)
    conn.rx_queue.put(
        MockMsg(
            "GLOBAL_POSITION_INT",
            lat=int(p_breach_lat * 1e7),
            lon=int(p_breach_lon * 1e7),
            relative_alt=15000,
            alt=599000,
        )
    )
    time.sleep(0.05)
    assert len(events) == 2
    assert events[1][0] == "BREACH"
    assert drone.abort_event.is_set()

    # Verify DO_SET_MODE (cmd 176) RTL was dispatched non-blocking
    assert len(conn.mav.sent_commands) == 1
    assert conn.mav.sent_commands[0]["command"] == mavutil.mavlink.MAV_CMD_DO_SET_MODE
