"""Distinguish delayed status updates from a mode that remains active."""

from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from demo import status_after_manual_exit


class ExitStatusTests(unittest.TestCase):
    def test_waits_for_delayed_mode_change_without_resending_commands(self):
        robot = Mock()
        robot.status.side_effect = [SimpleNamespace(state_code=n) for n in (7, 7, 3)]
        with patch("demo.time.sleep") as sleep:
            status = status_after_manual_exit(robot)
        self.assertEqual(status.state_code, 3)
        self.assertEqual(sleep.call_count, 2)
        self.assertEqual(robot.status.call_count, 3)
        robot.move_for.assert_not_called()
        robot.stop.assert_not_called()

    def test_persistent_manual_state_has_bounded_queries(self):
        robot = Mock()
        robot.status.return_value = SimpleNamespace(state_code=7)
        with patch("demo.time.sleep") as sleep:
            status = status_after_manual_exit(robot)
        self.assertEqual(status.state_code, 7)
        self.assertEqual(robot.status.call_count, 6)
        self.assertEqual(sleep.call_count, 5)
        robot.move_for.assert_not_called()
        robot.stop.assert_not_called()


if __name__ == "__main__":
    unittest.main()
