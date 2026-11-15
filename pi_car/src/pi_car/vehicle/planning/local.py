from pi_car.common.messages import Mode, MotorCommand


class LocalPlanner:
    """Extension point: stop → scan → avoid → safe rejoin → optional replan.

    Do not call vendor AUTO MATIC wholesale: it reverses without rear sensing.
    Needs footprint checks, servo settling, range freshness and bounded maneuvers.
    """

    def step(self, pose, ranges, path, navigation_grid):
        # Current milestone deliberately keeps the car stopped on an obstacle.
        return Mode.OBSTACLE, MotorCommand()
