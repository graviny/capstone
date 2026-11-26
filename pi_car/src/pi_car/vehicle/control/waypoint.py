import math
from pi_car.common.messages import MotorCommand


class WaypointFollower:
    def __init__(self, tolerance_m=.08, turn_threshold_rad=.55, output=.2):
        if tolerance_m <= 0 or not 0 < turn_threshold_rad < math.pi or not 0 < output <= 1:
            raise ValueError("Invalid follower settings")
        self.tolerance, self.turn_threshold, self.output = tolerance_m, turn_threshold_rad, output
        self.goal_tolerance = tolerance_m
        self.reset()

    def reset(self):
        self.index = 0
        self.pivoting = False

    def step(self, pose, path):
        while self.index < len(path):
            x, y = path[self.index]
            at_final_waypoint = self.index == len(path) - 1
            tolerance = self.goal_tolerance if at_final_waypoint else self.tolerance
            distance = math.hypot(x - pose.x, y - pose.y)
            if distance > tolerance:
                break
            self.index += 1
            self.pivoting = False
        if self.index == len(path):
            return MotorCommand(), True

        error = math.atan2(y - pose.y, x - pose.x) - pose.theta
        error = math.atan2(math.sin(error), math.cos(error))
        pivot_threshold = self.turn_threshold * .45 if self.pivoting else self.turn_threshold
        self.pivoting = abs(error) > pivot_threshold
        if self.pivoting:
            turn = math.copysign(self.output, error)
            return MotorCommand(-turn, turn), False

        speed = self.output
        if at_final_waypoint:
            # Keep enough output to turn the wheels as the vehicle approaches the goal.
            approach_output = min(speed, .20)
            approach_fraction = min(1., (distance - tolerance) / .045)
            speed = approach_output + (speed - approach_output) * approach_fraction
            # The bearing to a nearby goal is especially sensitive to position jitter.
            error *= min(1., distance / .05)

        correction = math.copysign(min(speed * .40, .14 * max(0., abs(error) - .04)), error)
        return MotorCommand(speed - max(0., correction), speed + min(0., correction)), False
