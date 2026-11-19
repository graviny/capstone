"""WES-facing adapter. Does not choose goals, allocate tasks or queue destinations."""


class CommandInterface:
    def __init__(self, fleet):
        self.fleet = fleet

    async def handle(self, request):
        if not isinstance(request, dict):
            raise ValueError("Expected a command object")
        kind = request.get("type")
        if kind == "MOVE":
            goal = request.get("goal")
            goal = goal if isinstance(goal, dict) else {}
            return await self.fleet.dispatch_goal(
                robot_id=request.get("robot_id"), x=goal.get("x"), y=goal.get("y"),
                command_id=request.get("command_id"), frame_id=request.get("frame_id"),
                map_version=request.get("map_version"))
        if kind == "STOP":
            return await self.fleet.stop_robot(request.get("robot_id"), request.get("command_id"))
        if kind == "STATUS":
            return self.fleet.snapshot()
        if kind == "INFO":
            m = self.fleet.grid.metadata
            return dict(frame_id=m.frame_id, map_version=m.version,
                        clearance_m=self.fleet.clearance, robot_ids=list(self.fleet.robots))
        if kind == "RESULT":
            return self.fleet.command_result(request.get("command_id"))
        raise ValueError("Expected MOVE, STOP, STATUS, INFO or RESULT")
