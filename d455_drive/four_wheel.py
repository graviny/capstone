"""WebSocket driving profile for the four-motor Adeept OrdinaryWheels chassis."""
from contextlib import nullcontext
import math
import time

import config as C
from pi_car.common.messages import MotorCommand
from pi_car.vehicle.hardware import DisabledMotor
from pi_car.vehicle.hardware.watchdog import WatchdogMotor

MAX_SPEED = 30.
STOP_DISTANCE_CM = 40.


def wheel_command(speed, steer):
    throttle = max(-MAX_SPEED, min(MAX_SPEED, speed)) / 100.
    turn = max(-1., min(1., steer / C.STEER_MAX_DEG))
    return MotorCommand(throttle * (1 - max(0., turn)), throttle * (1 + min(0., turn)))


class FourWheelCar:
    drive_type = "differential"

    def __init__(self, hardware, fake_dist, allow_no_echo=False):
        self.hardware = hardware
        self.fake_dist = fake_dist
        self.allow_no_echo = allow_no_echo
        driver = hardware.motor if hardware is not None else DisabledMotor()
        self.motor = WatchdogMotor(driver, timeout_s=.25)
        self.command = MotorCommand()

    def drive(self, speed, steer):
        self._apply(wheel_command(speed, steer))

    def drive_wheels(self, left, right):
        """Direct left/right output in percent (opposite signs rotate in place)."""
        self._apply(MotorCommand(*(max(-MAX_SPEED, min(MAX_SPEED, v)) / 100. for v in (left, right))))

    def _apply(self, command):
        if command == MotorCommand():
            if self.command != command:
                self.motor.stop()
        else:
            if self.command == MotorCommand():
                self.motor.arm()
            try:
                self.motor.apply(command)
            except RuntimeError:
                # The motor watchdog latched because the control loop stalled.
                # The command itself is still fresh (server.py checks that), so re-arm.
                self.motor.arm()
                self.motor.apply(command)
        self.command = command

    def distance_cm(self):
        if self.hardware is None:
            return self.fake_dist
        sensor = self.hardware.sensor
        reading = sensor.read_allow_clear() if self.allow_no_echo else sensor.read()
        if reading is None or not reading.usable(time.monotonic()) or abs(reading.angle_rad) > .05:
            return None
        return reading.distance_m * 100

    def battery_v(self):
        return None

    def close(self):
        self.motor.close()


def hardware_context(dry_run, vendor_dir, max_speed, servo_angle):
    if dry_run:
        return nullcontext(None)
    from pi_car.vehicle.hardware.live import AdeeptHardware
    return AdeeptHardware(vendor_dir, max_output=max_speed / 100, servo_angle=servo_angle)


class FrontStop:
    """Stop forward motion when the front range is invalid or below the threshold.

    Same interface as safety.Safety. Like the PiCar-B2, only forward motion is
    guarded (the sensor faces forward); reversing is the laptop's responsibility,
    which also lets the laptop back the car out of a blocked state.
    """
    phase = None

    def __init__(self, car):
        self.car = car
        self.state = "normal"
        self.dist = None

    def step(self, now, speed, steer):
        self.dist = self.car.distance_cm()
        valid = self.dist is not None and math.isfinite(self.dist) and self.dist > 0
        obstacle = not valid or self.dist <= STOP_DISTANCE_CM
        events = []
        if obstacle != (self.state == "blocked"):
            events.append({"type": "event", "event": "blocked" if obstacle else "clear",
                           "dist_cm": round(self.dist, 1) if valid else None})
        self.state = "blocked" if obstacle else "normal"
        if obstacle and speed > 0:
            return 0., 0., events
        return speed, steer, events
