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
        self.wheels = []
        if enabled:
            for channel in range(1, 5):
                motor = getattr(move_module, f"motor{channel}", None)
                polarity = getattr(move_module, f"M{channel}_Direction", None)
                if not hasattr(motor, "throttle") or polarity not in (-1, 1):
                    raise ValueError(f"Incompatible Move.py: motor {channel} or direction is unavailable")
                self.wheels.append((motor, polarity))

    def stop(self):
        self.move.motorStop()

    def apply(self, command):
        if not self.enabled:
            self.stop()
            if command.left or command.right:
                raise RuntimeError("Hardware driving is disabled")
            return
        try:
            # Move.setup() sets the shared PWM frequency. Move.Motor() would
            # put the PCA9685 to sleep and reset that frequency for each wheel.
            values = (command.left, command.left, command.right, command.right)
            for (motor, polarity), value in zip(self.wheels, values):
                motor.throttle = polarity * max(-self.max_output, min(self.max_output, value))
        except BaseException:
            self.stop()
            raise
