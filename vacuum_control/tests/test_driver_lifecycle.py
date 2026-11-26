"""Exercise the real installed miio lifecycle with all network I/O mocked."""

from contextlib import ExitStack
import unittest
from unittest.mock import patch
import warnings

from miio import RoborockVacuum
from miio.integrations.vacuum.roborock import vacuum as vacuum_module

from robot_driver import XiaomiRobot


class Harness:
    def __init__(self, state=7, move_error=None, stop_error=None):
        self.elapsed = 0.0
        self.calls = []
        self.state = state
        self.move_error = move_error
        self.stop_error = stop_error

    def sleep(self, seconds):
        self.elapsed += seconds

    def send(self, command, params=None):
        self.calls.append((command, params, self.elapsed))
        if command == "get_status":
            return [{"state": self.state, "error_code": 0, "battery": 80}]
        if command == "app_rc_move" and self.move_error:
            raise self.move_error
        if command == "app_rc_end" and self.stop_error:
            raise self.stop_error
        return ["ok"]

    def driver(self):
        # TEST-NET address, dummy token, and send() replacement: no real device.
        device = RoborockVacuum("192.0.2.1", "0" * 32, model="rockrobo.vacuum.v1")
        device.send = self.send
        return XiaomiRobot(device)


class DriverTests(unittest.TestCase):
    def setUp(self):
        self.context = ExitStack()
        self.addCleanup(self.context.close)
        self.context.enter_context(patch("socket.socket", side_effect=AssertionError(
            "Tests must not open network sockets"
        )))

    def run_motion(self, harness, **motion):
        with patch.object(vacuum_module.time, "sleep", side_effect=harness.sleep):
            return harness.driver().move_for(**motion)

    def test_short_motion_keeps_cli_post_move_wait(self):
        harness = Harness()
        result = self.run_motion(harness, rotation=30, velocity=0, duration_ms=500)
        move = next(c for c in harness.calls if c[0] == "app_rc_move")
        end = harness.calls[-1]
        self.assertEqual(result, ["ok"])
        self.assertEqual(move[1][0]["duration"], 500)
        self.assertEqual(move[1][0]["seqnum"], 1)
        self.assertEqual(move[2], 5.0)
        self.assertEqual(end[0], "app_rc_end")
        self.assertEqual(end[2] - move[2], 5.0)

    def test_five_second_forward_completes_before_manual_mode_ends(self):
        harness = Harness()
        self.assertEqual(self.run_motion(harness, duration_ms=5000), ["ok"])
        moves = [call for call in harness.calls if call[0] == "app_rc_move"]
        self.assertEqual(len(moves), 1)
        self.assertEqual(moves[0][1][0]["duration"], 5000)
        self.assertEqual(moves[0][2], 5.0)
        self.assertEqual(harness.calls[-1][0], "app_rc_end")
        self.assertEqual(harness.calls[-1][2] - moves[0][2], 6.0)

    def test_motion_error_still_requests_stop(self):
        harness = Harness(move_error=OSError("response lost"))
        with self.assertRaisesRegex(OSError, "response lost"):
            self.run_motion(harness)
        self.assertEqual(harness.calls[-1][0], "app_rc_end")

    def test_failed_cleanup_preserves_original_error(self):
        harness = Harness(move_error=OSError("move timeout"),
                          stop_error=OSError("stop timeout"))
        with warnings.catch_warnings(record=True) as caught:
            warnings.simplefilter("always")
            with self.assertRaisesRegex(OSError, "move timeout"):
                self.run_motion(harness)
        self.assertEqual(len(caught), 1)

    def test_failed_activation_never_claims_success_or_sends_motion(self):
        harness = Harness(state=8)
        with self.assertRaisesRegex(RuntimeError, "이동 명령이 전송되지"):
            self.run_motion(harness)
        self.assertFalse(any(c[0] == "app_rc_move" for c in harness.calls))
        self.assertEqual(harness.calls[-1][0], "app_rc_end")

    def test_interrupt_during_motion_requests_stop(self):
        harness = Harness(move_error=KeyboardInterrupt())
        with self.assertRaises(KeyboardInterrupt):
            self.run_motion(harness)
        self.assertEqual(harness.calls[-1][0], "app_rc_end")

    def test_invalid_motion_is_rejected_before_transmission(self):
        for options in ({"velocity": float("nan")}, {"velocity": 0.3},
                        {"rotation": 90}, {"duration_ms": 5001},
                        {"duration_ms": 5000, "velocity": -0.1}):
            with self.subTest(options=options):
                harness = Harness()
                with self.assertRaises(ValueError):
                    self.run_motion(harness, **options)
                self.assertEqual(harness.calls, [])

    def test_each_motion_restarts_device_sequence(self):
        harness = Harness()
        driver = harness.driver()
        with patch.object(vacuum_module.time, "sleep", side_effect=harness.sleep):
            driver.move_for()
            driver.move_for()
        sequence_numbers = [c[1][0]["seqnum"] for c in harness.calls
                            if c[0] == "app_rc_move"]
        self.assertEqual(sequence_numbers, [1, 1])


if __name__ == "__main__":
    unittest.main()
