from __future__ import annotations

import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch


SOURCE_DIR = Path(__file__).parents[1] / "src" / "ir-heater"
sys.path.insert(0, str(SOURCE_DIR))

from sequence_runner import (  # noqa: E402
    GrblCommunicationError,
    GrblController,
    GrblPositionUncertainError,
    RunMetadata,
    SequenceStep,
    run_sequence,
)


def controller_for_recovery() -> GrblController:
    controller = object.__new__(GrblController)
    controller._x_max = None
    controller._y_max = None
    controller._z_max = None
    controller._last_feedrate = None
    controller._last_reported_position = (0.0, 0.0, 0.0)
    controller._last_commanded_position = (0.0, 0.0, 0.0)
    controller._event_sink = None
    controller._recovery_state_sink = None
    controller._initialized = True
    controller._command_number = 0
    return controller


class GrblMotionRecoveryTests(unittest.TestCase):
    def test_linear_move_is_self_contained_and_retried_at_known_start(self) -> None:
        controller = controller_for_recovery()
        controller._send_line = Mock(side_effect=[TimeoutError("lost ok"), "ok"])
        controller._probe_status_for_recovery = Mock(
            return_value="<Idle|WPos:0.000,0.000,0.000>"
        )
        states: list[str] = []
        controller.set_recovery_state_sink(states.append)

        controller.send_move(19.3, 0.0, 0.0, 1000.0)

        command = "G1 X19.300 Y0.000 Z0.000 F1000.00"
        self.assertEqual(
            [call.args[0] for call in controller._send_line.call_args_list],
            [command, command],
        )
        self.assertEqual(states, ["started", "recovered"])
        self.assertEqual(controller._last_commanded_position, (19.3, 0.0, 0.0))

    def test_running_status_confirms_lost_ack_without_duplicate_move(self) -> None:
        controller = controller_for_recovery()
        controller._send_line = Mock(side_effect=TimeoutError("lost ok"))
        controller._probe_status_for_recovery = Mock(
            return_value="<Run|WPos:4.000,0.000,0.000>"
        )

        controller.send_move(19.3, 0.0, 0.0, 1000.0)

        self.assertEqual(controller._send_line.call_count, 1)
        self.assertEqual(controller._last_commanded_position, (19.3, 0.0, 0.0))

    def test_arc_is_not_resent_when_controller_is_idle_at_start(self) -> None:
        controller = controller_for_recovery()
        controller._send_line = Mock(side_effect=TimeoutError("lost ok"))
        controller._probe_status_for_recovery = Mock(
            return_value="<Idle|WPos:0.000,0.000,0.000>"
        )

        with self.assertRaisesRegex(GrblCommunicationError, "cannot be resent safely"):
            controller.send_arc(
                0.0, 0.0, 10.0, 0.0, 0.0, 5.0, 0.0, 100.0,
            )
        self.assertEqual(controller._send_line.call_count, 1)

    def test_implausible_recovered_position_aborts(self) -> None:
        controller = controller_for_recovery()
        with self.assertRaisesRegex(GrblPositionUncertainError, "implausible position"):
            controller._classify_recovery_status(
                "<Idle|WPos:200.000,50.000,0.000>",
                (0.0, 0.0, 0.0),
                (19.3, 0.0, 0.0),
            )

    def test_reconnect_resolution_uses_physical_usb_location(self) -> None:
        controller = controller_for_recovery()
        controller._serial_port = "/dev/ttyUSB2"
        controller._port_identity = {
            "device": "/dev/ttyUSB2",
            "serial_number": None,
            "location": "8-1.1.4",
        }
        ports = [
            SimpleNamespace(
                device="/dev/ttyUSB0", serial_number=None, location="8-1.1.2"
            ),
            SimpleNamespace(
                device="/dev/ttyUSB1", serial_number=None, location="8-1.1.4"
            ),
        ]
        with patch("sequence_runner._list_ports.comports", return_value=ports):
            self.assertEqual(controller._resolve_reconnect_port(), "/dev/ttyUSB1")

    def test_reset_banner_during_status_query_is_fatal(self) -> None:
        controller = controller_for_recovery()
        serial = Mock()
        serial.readline.return_value = b"Grbl 1.1h ['$' for help]\r\n"
        controller._serial = serial

        with self.assertRaisesRegex(GrblPositionUncertainError, "reset banner"):
            controller._read_status_report(timeout_s=0.01)


class _FakeDps:
    def __init__(self) -> None:
        self.output_states: list[int] = []

    def onoff(self, _mode: str, value: int) -> None:
        self.output_states.append(value)

    def voltage_set(self, _mode: str, _value: float) -> None:
        pass

    def current_set(self, _mode: str, _value: float) -> None:
        pass


class _FailingPrinter:
    def __init__(self) -> None:
        self.moves = 0
        self.events: list[dict[str, object]] = []
        self.disconnected = False

    def set_event_sink(self, sink) -> None:
        self.event_sink = sink

    def set_recovery_state_sink(self, sink) -> None:
        self.recovery_sink = sink

    def get_status(self) -> str:
        return "<Idle|WPos:0.000,0.000,0.000>"

    def send_move(self, *_args) -> None:
        self.moves += 1
        if self.moves == 2:
            raise GrblCommunicationError("injected lost connection")

    def feed_hold(self) -> None:
        pass

    def wait_for_hold(self, timeout_s: float) -> bool:
        return True

    def _log_event(self, event: str, **details: object) -> None:
        record = {"event": event, **details}
        self.events.append(record)
        if getattr(self, "event_sink", None) is not None:
            self.event_sink(record)

    def disconnect(self) -> None:
        self.disconnected = True


class SequenceFailureAccountingTests(unittest.TestCase):
    def test_failed_attempt_is_not_counted_and_untrusted_position_is_not_moved(self) -> None:
        steps = [
            SequenceStep(0.0, 1.0, 13.4, 1.0, 0.0, 0.0, 1000.0),
            SequenceStep(0.0, 1.0, 13.4, 2.0, 0.0, 0.0, 1000.0),
        ]
        dps = _FakeDps()
        printer = _FailingPrinter()
        metadata = RunMetadata(run_id="fault_injection")

        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaisesRegex(GrblCommunicationError, "injected"):
                run_sequence(
                    steps,
                    dps=dps,
                    printer=printer,
                    time_mode="step",
                    dry_run=False,
                    return_to_origin=True,
                    record_dir=Path(directory),
                    metadata=metadata,
                )

        self.assertEqual(metadata.steps_completed, 1)
        self.assertFalse(metadata.completed)
        self.assertFalse(metadata.gantry_position_trusted)
        self.assertEqual(printer.moves, 2)  # no third, automatic-origin move
        self.assertEqual(dps.output_states[-1], 0)
        self.assertTrue(printer.disconnected)


if __name__ == "__main__":
    unittest.main()
