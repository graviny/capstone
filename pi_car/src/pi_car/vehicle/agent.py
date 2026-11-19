"""Per-robot command executor. Planning and motor ownership stay on the Pi."""
import asyncio
import math
import time
from pi_car.common.messages import Mode, Pose
from pi_car.transport.journal import CommandJournal
from pi_car.transport.protocol import TERMINAL, identifier, make_message, validate
from pi_car.vehicle.hardware import DisabledMotor
from pi_car.vehicle.hardware.watchdog import WatchdogMotor
from pi_car.vehicle.planning import plan
from pi_car.vehicle.planning.line_of_sight import segment_is_free
from pi_car.vehicle.runtime import VehicleRuntime


class RobotAgent:
    def __init__(self, robot_id, grid, *, clearance_m=.2, motor=None, journal_path=":memory:",
                 pose_timeout_s=.3, watchdog_s=.25):
        self.robot_id = identifier(robot_id)
        self.grid, self.navigation = grid, grid.inflated(clearance_m)
        self.clearance = clearance_m
        self.motor = WatchdogMotor(motor if motor is not None else DisabledMotor(), watchdog_s)
        self.runtime = VehicleRuntime(self.motor, robot_id, grid.metadata.frame_id, pose_timeout_s)
        self.journal = CommandJournal(journal_path)
        self.pose = None
        self.pose_received = 0.
        self.front_range = None
        self.connected = False
        self.active = None
        self.goal = None
        self.planning = None
        self.generation = 0
        self.emit = lambda event: None
        self.moving_reported = False
        # Process restart must never resurrect a previously accepted path.
        for command_id, fingerprint, event in self.journal.events():
            if event["type"] not in TERMINAL:
                error = self.message("ERROR", command_id=command_id, reason="PROCESS_RESTARTED")
                self.journal.put(command_id, fingerprint, error)

    def message(self, kind, **fields):
        m = self.grid.metadata
        return make_message(kind, self.robot_id, m.frame_id, m.version, **fields)

    def fresh_pose(self):
        p = self.pose
        return (p is not None and p.valid and p.frame_id == self.grid.metadata.frame_id
                and 0 <= time.time()-p.timestamp <= self.runtime.pose_timeout
                and 0 <= time.monotonic()-self.pose_received <= self.runtime.pose_timeout)

    def _record(self, event, fingerprint=None):
        old = self.journal.get(event["command_id"])
        self.journal.put(event["command_id"], old[0] if old else fingerprint, event)
        self.emit(event)

    def _finish(self, kind, reason):
        self.runtime.halt()
        self.generation += 1
        active, self.active = self.active, None
        self.goal = None
        if active:
            self._record(self.message(kind, command_id=active, reason=reason))

    def disconnect(self):
        self.connected = False
        self.pose = None
        self._finish("ERROR", "CONNECTION_LOST")

    def connect(self):
        self.runtime.halt()
        self.connected, self.pose = True, None
        self.emit(self.message("READY"))
        for _, _, event in self.journal.events():
            if event["type"] in TERMINAL:
                self.emit(event)  # replay results only, never motion

    async def handle(self, message):
        validate(message)
        kind = message["type"]
        if (message["robot_id"] != self.robot_id or
                message["frame_id"] != self.grid.metadata.frame_id or
                message["map_version"] != self.grid.metadata.version):
            self._finish("ERROR", "IDENTITY_OR_MAP_MISMATCH")
            self.pose = None
            raise ValueError("Robot/frame/map mismatch")
        if kind == "POSE":
            self.pose = Pose(**{k: message[k] for k in (
                "robot_id", "x", "y", "theta", "timestamp", "frame_id", "valid")})
            self.pose_received = time.monotonic()
            if not self.fresh_pose():
                self._finish("ERROR", "POSE_INVALID_OR_STALE")
            return
        if kind not in {"GOAL", "STOP"}:
            raise ValueError("Unexpected PC command")
        command_id = message["command_id"]
        fingerprint = self.journal.fingerprint(message)
        previous = self.journal.get(command_id)
        if previous:
            if previous[0] != fingerprint:
                self._finish("ERROR", "COMMAND_ID_CONFLICT")
                raise ValueError("Command ID reused for a different payload")
            # Planning is not acceptance. Duplicate pending command waits for the
            # original planning task, rather than reporting a false acceptance.
            if previous[1]["type"] != "PENDING":
                self.emit(previous[1])
            return
        self._finish("CANCELLED", "STOP_REQUESTED" if kind == "STOP" else "SUPERSEDED")
        self.journal.put(command_id, fingerprint, dict(type="PENDING"))
        if kind == "STOP":
            self._record(self.message("STOPPED", command_id=command_id, reason="STOP_REQUESTED"))
            return
        self.active, self.goal = command_id, message["goal"]
        self.moving_reported = False
        if not self.connected or not self.fresh_pose():
            self._finish("ERROR", "POSE_UNAVAILABLE")
            return
        goal_xy = (self.goal["x"], self.goal["y"])
        if not self.navigation.is_free(self.navigation.world_to_cell(*goal_xy)):
            self._finish("ERROR", "GOAL_NOT_FREE")
            return
        generation = self.generation
        start = (self.pose.x, self.pose.y)
        self.planning = asyncio.create_task(self._plan(generation, start, goal_xy))

    async def _plan(self, generation, start, goal):
        try:
            path = await asyncio.to_thread(plan, self.navigation, start, goal)
            if generation != self.generation or self.active is None:
                return
            if not self.connected or not self.fresh_pose():
                self._finish("ERROR", "POSE_LOST_DURING_PLANNING")
                return
            if math.dist(start, (self.pose.x, self.pose.y)) > self.grid.metadata.resolution_m:
                self._finish("ERROR", "POSE_JUMP_DURING_PLANNING")
                return
            # A short clear segment need not turn toward every grid center.
            # Validate against all touched inflated cells before shortcutting.
            if segment_is_free(self.navigation,start,goal):
                path=[goal]
            elif path[-1] != goal:
                path.append(goal)
            self.runtime.set_path(path)
            self.runtime.follower.tolerance = min(.05, self.grid.metadata.resolution_m*.45)
            self.motor.arm()
            self._record(self.message("GOAL_ACCEPTED", command_id=self.active,
                                      goal=self.goal, waypoint_count=len(path)))
        except asyncio.CancelledError:
            raise
        except Exception as error:
            if generation == self.generation:
                self._finish("ERROR", f"PLANNING_FAILED: {error}")

    def tick(self):
        if not self.active:
            self.runtime.halt()
            return
        if not self.fresh_pose():
            self._finish("ERROR", "POSE_TIMEOUT")
            return
        if not self.runtime.path:  # planning pending, motors already stopped
            return
        try:
            mode = self.runtime.tick(self.pose, self.front_range, connected=self.connected)
            if mode == Mode.EMERGENCY:
                self._finish("ERROR", "SAFETY_INPUT_INVALID")
            elif mode == Mode.OBSTACLE:
                self._finish("BLOCKED", "ULTRASONIC_OBSTACLE")
            elif mode == Mode.ARRIVED:
                self._finish("ARRIVED", "GOAL_REACHED")
                self.emit(self.message("READY"))
            elif mode == Mode.NORMAL and not self.moving_reported:
                self.moving_reported = True
                self._record(self.message("MOVING", command_id=self.active, goal=self.goal))
        except Exception as error:
            self._finish("ERROR", f"CONTROL_FAILED: {error}")

    async def close(self):
        self.disconnect()
        if self.planning:
            self.planning.cancel()
            await asyncio.gather(self.planning, return_exceptions=True)
        self.motor.close()
        self.journal.close()
