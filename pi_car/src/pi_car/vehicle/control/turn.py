import math

from pi_car.common.messages import MotorCommand


class TurnStalled(RuntimeError):
    """The measured angle stopped approaching the requested rotation."""


class RelativeTurn:
    def __init__(self, heading, degrees, output, now):
        self.degrees = degrees
        self.target = math.radians(degrees)
        magnitude = abs(degrees)
        self.tolerance = math.radians(min(1.5, max(.7, magnitude * .07), magnitude * .45))
        self.output = output
        self.heading = heading
        self.swept = 0.
        self.best_error = abs(self.target)
        self.last_progress = now
        self.last_motion = now
        self.motion_angle = 0.
        self.approach_limit = output
        self.braking_since = None

    def step(self, heading, now):
        previous_error = self.target - self.swept
        delta = heading - self.heading
        self.swept += math.atan2(math.sin(delta), math.cos(delta))
        self.heading = heading
        error = self.target - self.swept
        remaining = abs(error)

        if remaining < self.best_error - math.radians(1.):
            self.best_error = remaining
            self.last_progress = now
        if abs(self.swept - self.motion_angle) >= math.radians(.6):
            self.motion_angle = self.swept
            self.last_motion = now

        overshot = error * previous_error < 0 and remaining > self.tolerance
        if overshot:
            self.approach_limit = max(self.output * .18, self.approach_limit * .60)

        if self.braking_since is not None:
            if now - self.braking_since < .16:
                return MotorCommand(), False
            if remaining <= self.tolerance:
                return MotorCommand(), True
            self.braking_since = None
        elif remaining <= self.tolerance or overshot:
            self.braking_since = now
            return MotorCommand(), False

        stalled_for = now - self.last_progress
        if stalled_for >= 2.:
            raise TurnStalled
        # Reduce correction after overshooting. Restore the configured output
        # briefly if the wheels need more torque to start moving again.
        fraction = min(1., remaining / math.radians(14.))
        speed = min(self.approach_limit, self.output * (.82 + .18 * fraction))
        if now - self.last_motion >= .55:
            speed = self.output
        speed = math.copysign(speed, error)
        return MotorCommand(-speed, speed), False
