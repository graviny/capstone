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
        for distance in (40., None):
            car.fake_dist = distance
            speed, steer, events = safety.step(0., 20., 10.)
            car.drive(speed, steer)
            self.assertEqual(car.command, MotorCommand())
            self.assertEqual(safety.state, "blocked")
            self.assertEqual(events, [])
