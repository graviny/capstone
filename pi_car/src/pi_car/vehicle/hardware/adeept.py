"""Adapters for the inspected 20260317 OrdinaryWheels vendor code.

Vendor modules are injected explicitly, never imported at package import time.
The caller owns Move.setup()/destroy(), servo lifecycle and the motor watchdog.
"""
import math
import time
from pi_car.vehicle.perception import RangeReading


class AdeeptUltrasonic:
    def __init__(self, ultra_module):
        self.ultra = ultra_module

    def read(self, angle_rad=0.):
        # Ultra.checkdist() is cm, not m. 200cm is the configured maximum and
        # cannot distinguish a real return from a missing echo: reject saturation.
        cm = float(self.ultra.checkdist())
        distance = cm / 100 if math.isfinite(cm) and 0 < cm < 200 else math.nan
        return RangeReading(distance, angle_rad, time.monotonic())


class AdeeptServo:
    def __init__(self, servo_controller):
        self.controller = servo_controller

    def point(self, angle_rad):
        degrees = math.degrees(angle_rad)
        if not math.isfinite(degrees) or not -45 <= degrees <= 45:
            raise ValueError("Initial scan is limited to ±45 degrees")
        # Vendor automaticProcessing: channel 0, +45 left, -45 right, 0 front.
        # Caller must stop motors, wait for settling, then get a NEW range sample.
        self.controller.moveAngle(0, degrees)


class AdeeptMotor:
    def __init__(self, move_module, *, enabled=False, max_output=.3):
        if not 0 < max_output <= 1:
            raise ValueError("max_output must be within (0, 1]")
        self.move, self.enabled, self.max_output = move_module, enabled, max_output

    def stop(self):
        self.move.motorStop()

    def apply(self, command):
        if not self.enabled:
            self.stop()
            if command.left or command.right:
                raise RuntimeError("Hardware driving is disabled")
            return
        try:
            # M1/M2 left, M3/M4 right; preserve per-wheel vendor polarity.
            for channel, value in ((1, command.left), (2, command.left),
                                   (3, command.right), (4, command.right)):
                polarity = getattr(self.move, f"M{channel}_Direction")
                direction = polarity if value >= 0 else -polarity
                self.move.Motor(channel, direction, min(abs(value), self.max_output)*100)
        except BaseException:
            self.stop()
            raise
