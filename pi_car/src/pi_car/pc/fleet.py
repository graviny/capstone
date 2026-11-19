"""External-command API and per-robot state, independent of TCP/RealSense/WES."""
import asyncio
from dataclasses import dataclass, field
import time
import uuid
from pi_car.transport.journal import CommandJournal
from pi_car.transport.protocol import TERMINAL, identifier, make_message, number


@dataclass
class RobotState:
    robot_id: str
    connected: bool = False
    pose: object = None
    pose_error: str | None = None
    command_id: str | None = None
    goal: dict | None = None
    status: str = "DISCONNECTED"
    last_communication: float = 0.
    sender: object = field(default=None, repr=False)


class FleetController:
    def __init__(self, grid, robot_ids, *, clearance_m=.2, journal_path=":memory:"):
        ids = [identifier(r) for r in robot_ids]
        if not ids or len(set(ids)) != len(ids):
            raise ValueError("Robot IDs must be nonempty and unique")
        self.grid, self.navigation = grid, grid.inflated(clearance_m)
        self.clearance = clearance_m
        self.robots = {r: RobotState(r) for r in ids}
        self.locks = {r: asyncio.Lock() for r in ids}
        self.journal = CommandJournal(journal_path)
        self.waiters = {}
        for command_id, fingerprint, event in self.journal.events():
            if event["type"] not in TERMINAL:
                aborted = self.message("ERROR", event["robot_id"], command_id=command_id,
                                       reason="SERVER_RESTARTED")
                self.journal.put(command_id, fingerprint, aborted)

    def message(self, kind, robot_id, **fields):
        m = self.grid.metadata
        return make_message(kind, robot_id, m.frame_id, m.version, **fields)

    def register(self, hello, sender):
        robot_id = hello["robot_id"]
        if robot_id not in self.robots:
            raise ValueError("Unknown robot")
        state = self.robots[robot_id]
        if state.connected:
            raise ValueError("Robot already connected")
        if (hello["frame_id"] != self.grid.metadata.frame_id or
                hello["map_version"] != self.grid.metadata.version or
                hello.get("clearance_m") != self.clearance):
            raise ValueError("Robot map/frame/clearance mismatch")
        state.connected, state.sender, state.pose = True, sender, None
        state.status, state.last_communication = "CONNECTED", time.time()

    def _store(self, event, fingerprint=None):
        key = event["command_id"]
        old = self.journal.get(key)
        self.journal.put(key, old[0] if old else fingerprint, event)
        if event["type"] in TERMINAL:
            waiter = self.waiters.pop(key, None)
            if waiter and not waiter.done():
                waiter.set_result(event)

    def disconnected(self, robot_id, sender):
        state = self.robots[robot_id]
        if state.sender is not sender:
            return
        state.connected, state.sender, state.pose = False, None, None
        state.status = "DISCONNECTED"
        if state.command_id:
            old = self.journal.get(state.command_id)
            if old and old[1]["type"] not in TERMINAL:
                self._store(self.message("ERROR", robot_id, command_id=state.command_id,
                                         reason="CONNECTION_LOST"))

    def status_received(self, event):
        state = self.robots[event["robot_id"]]
        state.last_communication = time.time()
        if event["type"] == "READY":
            if not state.command_id or self.journal.get(state.command_id)[1]["type"] in TERMINAL:
                state.status = "IDLE"
            return
        old = self.journal.get(event["command_id"])
        if not old:  # old durable result from a Pi whose PC journal was replaced
            return
        if old[1]["robot_id"] != state.robot_id:
            raise ValueError("Result belongs to another robot")
        if old[1]["type"] in TERMINAL:
            return  # terminal result is immutable, including duplicate ARRIVED
        self._store(event)
        if state.command_id == event["command_id"]:
            state.status = event["type"]

    async def _dispatch(self, kind, robot_id, command_id, **fields):
        identifier(robot_id)
        identifier(command_id)
        if robot_id not in self.robots:
            raise ValueError("Unknown robot")
        message = self.message(kind, robot_id, command_id=command_id, **fields)
        fingerprint = self.journal.fingerprint(message)
        async with self.locks[robot_id]:
            previous = self.journal.get(command_id)
            if previous:
                if previous[0] != fingerprint:
                    raise ValueError("Command ID reused for another payload/robot")
                return previous[1]
            state = self.robots[robot_id]
            # API/journal state only. Acceptance comes exclusively from the Pi.
            pending = dict(message, type="PENDING")
            self._store(pending, fingerprint)
            if not state.connected:
                error = self.message("ERROR", robot_id, command_id=command_id, reason="NOT_CONNECTED")
                self._store(error)
                return error
            state.command_id, state.goal, state.status = command_id, fields.get("goal"), "PENDING"
            try:
                await state.sender.send(message)
            except (OSError, ConnectionError, asyncio.TimeoutError):
                self.disconnected(robot_id, state.sender)
            return self.journal.get(command_id)[1]

    async def dispatch_goal(self, robot_id, x, y, command_id, *, frame_id=None, map_version=None):
        """Called only by an external requester; no goal queue or automatic scheduling."""
        try:
            identifier(command_id); number(x); number(y)
            if frame_id is not None and frame_id != self.grid.metadata.frame_id:
                raise ValueError("Frame mismatch")
            if map_version is not None and map_version != self.grid.metadata.version:
                raise ValueError("Map version mismatch")
            if not self.navigation.is_free(self.navigation.world_to_cell(x, y)):
                raise ValueError("Goal is outside free navigation space")
        except ValueError:
            # A rejected replacement must not leave the previous motion running.
            if robot_id in self.robots and self.robots[robot_id].connected:
                await self.stop_robot(robot_id, "safety."+uuid.uuid4().hex)
            raise
        return await self._dispatch("GOAL", robot_id, command_id, goal=dict(x=x, y=y))

    async def stop_robot(self, robot_id, command_id):
        return await self._dispatch("STOP", robot_id, command_id)

    async def publish_pose(self, robot_id, pose):
        state = self.robots[robot_id]
        valid = (pose is not None and pose.valid and pose.robot_id == robot_id
                 and pose.frame_id == self.grid.metadata.frame_id
                 and 0 <= time.time()-pose.timestamp <= .3)
        state.pose = pose if valid else None
        if pose is not None and not valid:
            state.pose_error='POSE INVALID / WRONG FRAME / CAPTURE AGE OVER 0.3s'
        if not state.connected:
            return
        fields = dict(x=pose.x, y=pose.y, theta=pose.theta, timestamp=pose.timestamp) if valid else dict(
            x=0., y=0., theta=0., timestamp=time.time())
        try:
            await state.sender.send(self.message("POSE", robot_id, valid=bool(valid), **fields))
        except (OSError, ConnectionError, asyncio.TimeoutError):
            self.disconnected(robot_id, state.sender)

    def command_result(self, command_id):
        entry = self.journal.get(command_id)
        if not entry:
            raise ValueError("Unknown command ID")
        return entry[1]

    async def wait_result(self, command_id, timeout_s=60.):
        event = self.command_result(command_id)
        if event["type"] in TERMINAL:
            return event
        waiter = self.waiters.setdefault(command_id, asyncio.get_running_loop().create_future())
        return await asyncio.wait_for(asyncio.shield(waiter), timeout_s)

    def snapshot(self):
        return {r: dict(robot_id=r, connected=s.connected,
                        pose=s.pose.to_dict() if s.pose else None, command_id=s.command_id,
                        pose_error=s.pose_error,
                        goal=s.goal, status=s.status, last_communication=s.last_communication,
                        last_result=(self.journal.get(s.command_id)[1] if s.command_id else None))
                for r, s in self.robots.items()}

    def close(self):
        self.journal.close()
