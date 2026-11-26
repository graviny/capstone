import time
import types
import unittest

from four_wheel import FourWheelCar, FrontStop, wheel_command
from pi_car.common.messages import MotorCommand


class FourWheelTests(unittest.TestCase):
    def test_drive_direction_cap_and_zero_output(self):
        self.assertEqual(wheel_command(100, 0), MotorCommand(.30, .30))
        self.assertAlmostEqual(wheel_command(20, 15).left, .10)
        self.assertAlmostEqual(wheel_command(20, 15).right, .20)
        self.assertAlmostEqual(wheel_command(20, -15).left, .20)
        self.assertAlmostEqual(wheel_command(20, -15).right, .10)
        self.assertEqual(wheel_command(0, 30), MotorCommand())

    def test_close_or_invalid_range_stops_without_autonomous_motion(self):
        car = FourWheelCar(None, 100.)
        self.addCleanup(car.close)
        safety = FrontStop(car)
        speed, steer, _ = safety.step(0., 20., 10.)
        car.drive(speed, steer)
        self.assertGreater(car.command.right, 0)
        expected_events = {40.: [{"type": "event", "event": "blocked", "dist_cm": 40.}], None: []}
        for distance in (40., None):
            car.fake_dist = distance
            speed, steer, events = safety.step(0., 20., 10.)
            car.drive(speed, steer)
            self.assertEqual(car.command, MotorCommand())
            self.assertEqual(safety.state, "blocked")
            self.assertEqual(events, expected_events[distance])

    def test_reverse_is_allowed_while_blocked_and_clear_event(self):
        car = FourWheelCar(None, 20.)
        self.addCleanup(car.close)
        safety = FrontStop(car)
        speed, _, events = safety.step(0., -20., 0.)
        self.assertEqual(speed, -20.)
        self.assertEqual([e["event"] for e in events], ["blocked"])
        speed, _, events = safety.step(0., 0., 0.)
        self.assertEqual((safety.state, events), ("blocked", []))
        car.fake_dist = 100.
        speed, _, events = safety.step(0., 20., 0.)
        self.assertEqual(speed, 20.)
        self.assertEqual(events, [{"type": "event", "event": "clear", "dist_cm": 100.}])

    def test_no_echo_is_clear_only_when_allowed(self):
        sensor = types.SimpleNamespace(read=lambda: None, read_allow_clear=lambda: types.SimpleNamespace(
            distance_m=2., angle_rad=0., usable=lambda now: True))
        hardware = types.SimpleNamespace(sensor=sensor, motor=None)
        car = FourWheelCar.__new__(FourWheelCar)
        car.hardware, car.allow_no_echo = hardware, False
        self.assertIsNone(car.distance_cm())
        car.allow_no_echo = True
        self.assertEqual(car.distance_cm(), 200.)

    def test_stalled_loop_rearms_instead_of_crashing(self):
        car = FourWheelCar(None, 100.)
        self.addCleanup(car.close)
        car.drive(20, 0)
        time.sleep(.35)          # motor watchdog (0.25 s) latches
        car.drive(20, 0)         # must not raise
        self.assertEqual(car.command, wheel_command(20, 0))
