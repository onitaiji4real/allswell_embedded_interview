"""Exercise Part 3's safety cleanup when its expected test path fails."""

from __future__ import annotations

import threading
from types import SimpleNamespace
from typing import Any

import part3_failsafe
from starter.drone import DroneError, MissionAborted


class FakeDrone:
    instances: list["FakeDrone"] = []

    def __init__(self, **kwargs: Any) -> None:
        self.log = kwargs["log"]
        self.abort_event = threading.Event()
        self.telemetry = SimpleNamespace(mode="GUIDED", armed=True)
        self.listener: Any = None
        self.landing_waited = False
        self.param_should_fail = False
        FakeDrone.instances.append(self)

    def __enter__(self) -> "FakeDrone":
        return self

    def __exit__(self, *args: Any) -> None:
        pass

    def add_listener(self, callback: Any) -> None:
        self.listener = callback

    def abort_and_request_mode_nowait(self, mode_name: str) -> bool:
        if self.abort_event.is_set():
            return False
        self.abort_event.set()
        self.telemetry.mode = mode_name
        return True

    def set_param(self, name: str, value: float, **kwargs: Any) -> float:
        if self.param_should_fail:
            raise DroneError("PARAM_VALUE timeout")
        return value

    def set_mode(self, mode_name: str, **kwargs: Any) -> None:
        self.telemetry.mode = mode_name

    def wait_landed_disarmed(self, **kwargs: Any) -> None:
        self.landing_waited = True


def test_parameter_confirmation_failure_still_waits_for_landing(monkeypatch: Any) -> None:
    FakeDrone.instances.clear()
    monkeypatch.setattr(part3_failsafe, "Drone", FakeDrone)
    monkeypatch.setattr(part3_failsafe, "takeoff_sequence", lambda *args, **kwargs: None)
    monkeypatch.setattr(part3_failsafe.sys, "argv", ["part3_failsafe.py", "--fault-delay", "0"])

    def aborting_mission(drone: FakeDrone, *, on_leg_start: Any, **kwargs: Any) -> None:
        drone.param_should_fail = True
        on_leg_start(1, "B", (0.0, 0.0))
        drone.listener(SimpleNamespace(get_type=lambda: "SYS_STATUS", voltage_battery=10500))
        raise MissionAborted("low voltage")

    monkeypatch.setattr(part3_failsafe, "fly_square_mission", aborting_mission)
    assert part3_failsafe.main() == 1
    assert FakeDrone.instances[-1].landing_waited


def test_unaborted_square_still_returns_home_before_failing(monkeypatch: Any) -> None:
    FakeDrone.instances.clear()
    monkeypatch.setattr(part3_failsafe, "Drone", FakeDrone)
    monkeypatch.setattr(part3_failsafe, "takeoff_sequence", lambda *args, **kwargs: None)
    monkeypatch.setattr(part3_failsafe.sys, "argv", ["part3_failsafe.py", "--fault-delay", "20"])

    def completed_mission(drone: FakeDrone, *, on_leg_start: Any, **kwargs: Any) -> None:
        on_leg_start(1, "B", (0.0, 0.0))

    monkeypatch.setattr(part3_failsafe, "fly_square_mission", completed_mission)
    assert part3_failsafe.main() == 1
    assert FakeDrone.instances[-1].telemetry.mode == "RTL"
    assert FakeDrone.instances[-1].landing_waited


def test_landing_error_waits_for_injection_confirmation(monkeypatch: Any) -> None:
    started = threading.Event()
    release = threading.Event()
    landing_attempted = threading.Event()

    class LandingErrorDrone(FakeDrone):
        param_completed = False

        def set_param(self, name: str, value: float, **kwargs: Any) -> float:
            started.set()
            assert release.wait(2)
            self.param_completed = True
            return value

        def wait_landed_disarmed(self, **kwargs: Any) -> None:
            self.landing_waited = True
            landing_attempted.set()
            raise DroneError("landing telemetry timeout")

    monkeypatch.setattr(part3_failsafe, "Drone", LandingErrorDrone)
    monkeypatch.setattr(part3_failsafe, "takeoff_sequence", lambda *args, **kwargs: None)
    monkeypatch.setattr(part3_failsafe.sys, "argv", ["part3_failsafe.py", "--fault-delay", "0"])

    def aborting_mission(drone: FakeDrone, *, on_leg_start: Any, **kwargs: Any) -> None:
        on_leg_start(1, "B", (0.0, 0.0))
        assert started.wait(2)
        drone.listener(SimpleNamespace(get_type=lambda: "SYS_STATUS", voltage_battery=10500))
        raise MissionAborted("low voltage")

    monkeypatch.setattr(part3_failsafe, "fly_square_mission", aborting_mission)
    result: list[int] = []
    worker = threading.Thread(target=lambda: result.append(part3_failsafe.main()))
    worker.start()
    assert started.wait(2)
    assert landing_attempted.wait(2)
    worker.join(0.05)
    assert worker.is_alive()  # It must still be joining the pending parameter confirmation.
    release.set()
    worker.join(2)
    assert not worker.is_alive()
    assert result == [1]
    assert LandingErrorDrone.instances[-1].param_completed
