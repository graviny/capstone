import math
from pi_car.common.messages import MotorCommand


class WaypointFollower:
    def __init__(self, tolerance_m=.08, turn_threshold_rad=.25, output=.2):
        if tolerance_m <= 0 or not 0 < turn_threshold_rad < math.pi or not 0 < output <= 1:
            raise ValueError("Invalid follower settings")
        self.tolerance, self.turn_threshold, self.output = tolerance_m, turn_threshold_rad, output
        self.index = 0

    def reset(self):
        self.index = 0

    def step(self, pose, path):
        while self.index < len(path):
            x, y = path[self.index]
            if math.hypot(x-pose.x, y-pose.y) > self.tolerance:
                break
            self.index += 1
        if self.index == len(path):
            return MotorCommand(), True
        x, y = path[self.index]
        error = math.atan2(y-pose.y, x-pose.x) - pose.theta
        error = math.atan2(math.sin(error), math.cos(error))
        if abs(error) > self.turn_threshold:
            turn = math.copysign(self.output, error)
            return MotorCommand(-turn, turn), False
        return MotorCommand(self.output, self.output), False
