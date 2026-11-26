import time
from pi_car.common.messages import Mode
from pi_car.vehicle.control.turn import RelativeTurn
from pi_car.vehicle.control.waypoint import WaypointFollower


class VehicleRuntime:
    """Control-loop building block shared by offline checks and RobotAgent.

    RobotAgent invalidates motion on disconnect and wraps the motor in an
    independent thread watchdog. Hardware fail-off is still needed for OS stalls.
    """
    def __init__(self, motor, robot_id, frame_id, pose_timeout_s=.3,
                 stop_distance_m=.4, range_timeout_s=.2, *, drive_output=.2):
        self.motor, self.robot_id, self.frame_id = motor, robot_id, frame_id
        self.pose_timeout, self.stop_distance = pose_timeout_s, stop_distance_m
        self.range_timeout = range_timeout_s
        self.mode, self.path = Mode.IDLE, []
        self.follower = WaypointFollower(output=drive_output)
        self.turn = None
        self.safety_error = None

    def set_path(self, path):
        self.halt()
        self.path = list(path)

    def set_turn(self, heading, degrees):
        self.halt()
        self.turn = RelativeTurn(heading, degrees, self.follower.output, time.monotonic())

    def halt(self):
        """Invalidate motion; reconnection or fresh sensors cannot restore this path."""
        self.motor.stop()
        self.path = []
        self.turn = None
        self.follower.reset()
        self.mode = Mode.IDLE

    def tick(self, pose, front_range, *, connected, now_unix=None, now_monotonic=None):
        now_unix = time.time() if now_unix is None else now_unix
        now_monotonic = time.monotonic() if now_monotonic is None else now_monotonic
        valid_pose = (pose is not None and pose.valid and pose.robot_id == self.robot_id
                      and pose.frame_id == self.frame_id
                      and 0 <= now_unix-pose.timestamp <= self.pose_timeout)
        self.safety_error = None
        try:
            if not connected or not valid_pose:
                self.safety_error = "POSE_INVALID_OR_STALE" if connected else "CONNECTION_LOST"
                self.mode = Mode.EMERGENCY
                self.motor.stop()
            elif (front_range is None or not front_range.usable(now_monotonic, self.range_timeout)
                  or abs(front_range.angle_rad) > .05):
                self.safety_error = "ULTRASONIC_INPUT_INVALID"
                self.mode = Mode.EMERGENCY
                self.motor.stop()
            elif front_range.distance_m <= self.stop_distance:
                self.mode = Mode.OBSTACLE
                self.motor.stop()
            elif self.turn is None and not self.path:
                self.mode = Mode.IDLE
                self.motor.stop()
            else:
                if self.turn is None:
                    command, arrived = self.follower.step(pose, self.path)
                else:
                    command, arrived = self.turn.step(pose.theta, now_monotonic)
                self.mode = Mode.ARRIVED if arrived else Mode.NORMAL
                self.motor.apply(command)
        except BaseException:
            self.mode = Mode.EMERGENCY
            self.motor.stop()
            raise
        return self.mode
