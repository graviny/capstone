"""PC ws-robot ↔ d455_drive/server.py --four-wheel (dry-run) end to end.

Mock localization integrates the car server's actual wheel command, so a goal
travels fleet → RobotAgent on the PC → WebSocket "wheels" → FourWheelCar.
"""
import asyncio
import math
from pathlib import Path
import sys
import time
import unittest

try:
    import websockets  # noqa: F401
except ImportError:  # pragma: no cover
    websockets = None

from pi_car.common.messages import MotorCommand, Pose
from pi_car.mock import mock_grid
from pi_car.pc.fleet import FleetController
from pi_car.pc.server import FleetTCPServer
from pi_car.pc.ws_robot import WebSocketCar, WebSocketWheels
from pi_car.vehicle.agent import RobotAgent
from pi_car.vehicle.client import RobotClient

D455 = Path(__file__).resolve().parents[2] / "d455_drive"


@unittest.skipIf(websockets is None, "websockets not installed")
class WebSocketRobotTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        sys.path.insert(0, str(D455))
        import server as car_server
        from four_wheel import FourWheelCar, FrontStop
        from websockets.asyncio.server import serve
        self.car = FourWheelCar(None, 150.)
        self.ctl = car_server.Controller(self.car, 30., FrontStop, "picar4wd")
        self.car_ws = await serve(car_server.make_handler(self.ctl), "127.0.0.1", 0)
        car_port = self.car_ws.sockets[0].getsockname()[1]
        self.tasks = [asyncio.create_task(car_server.control_loop(self.ctl))]

        self.grid = mock_grid()
        self.fleet = FleetController(self.grid, ["1"], clearance_m=.2)
        self.server = await FleetTCPServer(self.fleet, "127.0.0.1", 0, timeout_s=.6, heartbeat_s=.1).start()
        self.wheels = WebSocketWheels(asyncio.get_running_loop(), .3)
        self.link = WebSocketCar(f"ws://127.0.0.1:{car_port}", self.wheels, expected_robot_id="picar4wd")
        self.agent = RobotAgent("1", self.grid, motor=self.wheels, clearance_m=.2, drive_output=.3)
        self.agent.runtime.range_timeout = .4
        self.client = RobotClient(self.agent, "127.0.0.1", self.server.port, timeout_s=.6,
                                  heartbeat_s=.1, reconnect_s=.05, range_provider=self.link.front_range)
        self.position = [1., 1., 0.]
        self.tasks += [asyncio.create_task(self.link.run()), asyncio.create_task(self.client.run()),
                       asyncio.create_task(self._world())]
        for _ in range(200):
            if self.fleet.robots["1"].connected and self.link.range is not None:
                break
            await asyncio.sleep(.02)

    async def _world(self):
        previous = time.monotonic()
        while True:
            now = time.monotonic()
            dt, previous = min(now - previous, .05), now
            left, right = self.car.command.left, self.car.command.right
            x, y, theta = self.position
            # Roughly the measured Adeept rates: 0.30 output ≈ 0.12 m/s, pivot ≈ 35°/s.
            theta += (right - left) * dt
            v = (left + right) / 2 * .4
            self.position = [x + v * math.cos(theta) * dt, y + v * math.sin(theta) * dt, theta]
            await self.fleet.publish_pose("1", Pose("1", *self.position, time.time(),
                                                    self.grid.metadata.frame_id))
            await asyncio.sleep(.015)

    async def asyncTearDown(self):
        self.link.running = False
        await self.client.close()
        for task in self.tasks:
            task.cancel()
        await asyncio.gather(*self.tasks, return_exceptions=True)
        self.car_ws.close()
        await self.car_ws.wait_closed()
        await self.server.close()
        await self.agent.close()
        self.fleet.close()
        self.car.close()
        sys.path.remove(str(D455))

    async def test_clicked_goal_drives_the_websocket_car_to_arrival(self):
        await self.fleet.dispatch_goal("1", 1.4, 1.25, "click")
        result = await self.fleet.wait_result("click", 20.)
        self.assertEqual((result["type"], result["reason"]), ("ARRIVED", "GOAL_REACHED"))
        self.assertLess(math.dist(self.position[:2], (1.4, 1.25)), .06)
        await asyncio.sleep(.15)
        self.assertEqual(self.car.command, MotorCommand())

    async def test_turn_button_rotates_in_place_over_websocket(self):
        await self.fleet.dispatch_turn("1", 90., "turn")
        result = await self.fleet.wait_result("turn", 15.)
        self.assertEqual(result["type"], "ARRIVED", result)
        self.assertAlmostEqual(math.degrees(self.position[2]), 90., delta=3.)

    async def test_front_obstacle_from_car_status_blocks_goal(self):
        self.car.fake_dist = 30.
        await asyncio.sleep(.25)
        await self.fleet.dispatch_goal("1", 2.0, 1.0, "blocked")
        result = await self.fleet.wait_result("blocked", 5.)
        self.assertEqual(result["type"], "BLOCKED")
        self.assertEqual(self.car.command, MotorCommand())

    async def test_car_link_loss_stops_and_fails_the_command(self):
        await self.fleet.dispatch_goal("1", 3.0, 1.0, "loss")
        for _ in range(100):
            if self.car.command != MotorCommand():
                break
            await asyncio.sleep(.02)
        self.assertNotEqual(self.car.command, MotorCommand())
        self.link.running = False
        self.tasks[1].cancel()
        result = await self.fleet.wait_result("loss", 5.)
        self.assertEqual((result["type"], result["reason"]), ("ERROR", "ULTRASONIC_INPUT_INVALID"))
        await asyncio.sleep(.7)  # car server's own 0.5 s command timeout
        self.assertEqual(self.car.command, MotorCommand())


if __name__ == "__main__":
    unittest.main()
