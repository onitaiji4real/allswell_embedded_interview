"""Verify failure recovery happens before the MAVLink transport closes."""

from __future__ import annotations

from types import SimpleNamespace
from typing import Any

import pytest

import part1_takeoff
import part2_square
from starter.drone import DroneError
from starter.mission import flight_guard


class RecoveryDrone:
    instances: list["RecoveryDrone"] = []
    takeoff_error: BaseException = DroneError("takeoff failed")

    def __init__(self, **kwargs: Any) -> None:
        self.log = kwargs["log"]
        self.telemetry = SimpleNamespace(armed=True, mode="GUIDED")
        self.closed = False
        self.landing_waited = False
        RecoveryDrone.instances.append(self)

    def __enter__(self) -> "RecoveryDrone":
        return self

    def __exit__(self, *args: Any) -> None:
        self.closed = True

    def wait_ready_to_arm(self, **kwargs: Any) -> None:
        pass

    def set_mode(self, mode: str, **kwargs: Any) -> None:
        assert not self.closed
        self.telemetry.mode = mode

    def arm(self, **kwargs: Any) -> None:
        pass

    def takeoff(self, **kwargs: Any) -> None:
        raise self.takeoff_error

    def wait_landed_disarmed(self, **kwargs: Any) -> None:
        assert not self.closed
        self.landing_waited = True


@pytest.mark.parametrize("module", [part1_takeoff, part2_square])
@pytest.mark.parametrize(
    ("failure", "expected_exit"),
    [(DroneError("takeoff failed"), 1), (KeyboardInterrupt(), 130)],
)
def test_failed_takeoff_recovers_before_connection_closes(
    monkeypatch: Any, module: Any, failure: BaseException, expected_exit: int
) -> None:
    RecoveryDrone.instances.clear()
    monkeypatch.setattr(RecoveryDrone, "takeoff_error", failure)
    monkeypatch.setattr(module, "Drone", RecoveryDrone)
    monkeypatch.setattr(module.sys, "argv", [module.__name__ + ".py"])
    assert module.main() == expected_exit
    drone = RecoveryDrone.instances[-1]
    assert drone.telemetry.mode == "RTL"
    assert drone.landing_waited
    assert drone.closed


def test_recovery_waits_for_landing_after_rtl_ack_failure() -> None:
    class AckFailureDrone(RecoveryDrone):
        def set_mode(self, mode: str, **kwargs: Any) -> None:
            raise DroneError("RTL ACK timeout")

        def request_mode_nowait(self, mode: str) -> None:
            assert not self.closed
            self.telemetry.mode = mode

    drone = AckFailureDrone(log=part1_takeoff.logger)
    with pytest.raises(DroneError, match="flight failed"):
        with drone, flight_guard(drone):
            raise DroneError("flight failed")
    assert drone.telemetry.mode == "RTL"
    assert drone.landing_waited
    assert drone.closed
