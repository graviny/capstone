"""Configured output reaches motor commands while existing stop guards remain."""
import argparse
from contextlib import nullcontext
import math
from pathlib import Path
import tempfile
import time
from types import SimpleNamespace
import unittest
from unittest.mock import AsyncMock, patch

from pi_car.common.messages import MotorCommand
from pi_car.fleet_cli import add_commands, run
from pi_car.mock import mock_grid
from pi_car.vehicle.agent import RobotAgent
from pi_car.vehicle.control.turn import RelativeTurn, TurnStalled
from pi_car.vehicle.control.waypoint import WaypointFollower
from pi_car.vehicle.perception import RangeReading


class RecordingMotor:
    def __init__(self):
        self.command=MotorCommand()

    def stop(self):
        self.command=MotorCommand()

    def apply(self, command):
        self.command=command


class WaypointFollowerTests(unittest.TestCase):
    def test_small_heading_changes_near_goal_keep_both_wheels_forward(self):
        follower = WaypointFollower(tolerance_m=.01, output=.25)
        commands = []
        for heading in (-.24, -.26, -.24):
            command, arrived = follower.step(SimpleNamespace(x=0., y=0., theta=heading), [(.018, 0.)])
            self.assertFalse(arrived)
            self.assertGreater(command.left, 0.)
            self.assertGreater(command.right, command.left)
            self.assertLess(command.right, .25)
            commands.append(command)
        self.assertAlmostEqual(commands[0].left, commands[1].left, delta=.002)

    def test_goal_has_its_own_tolerance_and_only_final_waypoint_slows_down(self):
        follower = WaypointFollower(tolerance_m=.04, output=.25)
        follower.goal_tolerance = .01
        path = [(.05, 0.), (.10, 0.)]
        command, arrived = follower.step(SimpleNamespace(x=0., y=0., theta=0.), path)
        self.assertEqual(command, MotorCommand(.25, .25))
        self.assertFalse(arrived)
        command, arrived = follower.step(SimpleNamespace(x=.085, y=0., theta=0.), path)
        self.assertFalse(arrived)
        self.assertGreater(command.left, 0.)
        self.assertLess(command.left, .25)
        self.assertEqual(follower.step(SimpleNamespace(x=.092, y=0., theta=0.), path),
                         (MotorCommand(), True))


class RelativeTurnTests(unittest.TestCase):
    def test_wraps_heading_and_brakes_until_it_settles(self):
        turn = RelativeTurn(math.radians(175.), 25., .30, 0.)
        self.assertEqual(turn.step(math.radians(175.), .02), (MotorCommand(-.30, .30), False))
        self.assertEqual(turn.step(math.radians(-160.), 1.), (MotorCommand(), False))
        self.assertEqual(turn.step(math.radians(-159.), 1.17), (MotorCommand(), True))
        self.assertAlmostEqual(math.degrees(turn.swept), 26.)

    def test_reports_stall_when_heading_does_not_advance(self):
        turn = RelativeTurn(0., -25., .30, 0.)
        self.assertEqual(turn.step(0., .1), (MotorCommand(.30, -.30), False))
        with self.assertRaises(TurnStalled):
            turn.step(0., 2.)


class DriveOutputTests(unittest.IsolatedAsyncioTestCase):
    async def test_legacy_lower_hardware_limit_still_starts_with_capped_default_output(self):
        parser=argparse.ArgumentParser()
        add_commands(parser.add_subparsers(dest='command',required=True))
        hardware=SimpleNamespace(motor=RecordingMotor(),sensor=SimpleNamespace(read=lambda:None))
        with tempfile.TemporaryDirectory() as directory:
            mock_grid().save(Path(directory)/'map')
            args=parser.parse_args(['robot-client','--robot-id','A','--host','localhost',
                                   '--map',str(Path(directory)/'map'),'--journal',':memory:',
                                   '--enable-motors','--vendor-dir','unused','--max-output','0.15'])
            with patch('pi_car.vehicle.hardware.live.AdeeptHardware',
                       return_value=nullcontext(hardware)), \
                 patch('pi_car.vehicle.client.RobotClient') as client:
                client.return_value.run=AsyncMock()
                client.return_value.close=AsyncMock()
                await run(args)
                agent=client.call_args.args[0]
                self.assertEqual(agent.runtime.follower.output,.15)
                self.assertEqual(hardware.motor.command,MotorCommand())

    async def test_configured_agent_output_moves_stops_for_obstacle_and_does_not_resume(self):
        motor=RecordingMotor()
        agent=RobotAgent('A',mock_grid(),motor=motor,drive_output=.25)
        try:
            agent.connect()
            await agent.handle(agent.message('POSE',valid=True,x=1.,y=1.,theta=0.,timestamp=time.time()))
            await agent.handle(agent.message('GOAL',command_id='faster',goal=dict(x=1.5,y=1.)))
            await agent.planning
            agent.front_range=RangeReading(1.,0.,time.monotonic())
            agent.tick()
            self.assertEqual(motor.command,MotorCommand(.25,.25))
            agent.front_range=RangeReading(.3,0.,time.monotonic())
            agent.tick()
            self.assertEqual(motor.command,MotorCommand())
            self.assertEqual(agent.journal.get('faster')[1]['type'],'BLOCKED')
            agent.front_range=RangeReading(1.,0.,time.monotonic())
            agent.tick()
            self.assertEqual(motor.command,MotorCommand())
            self.assertIsNone(agent.active)
        finally:
            await agent.close()

    async def test_invalid_cli_outputs_fail_before_loading_map_or_opening_hardware(self):
        parser=argparse.ArgumentParser()
        add_commands(parser.add_subparsers(dest='command',required=True))
        for drive,maximum in [('0.31','0.30'),('0.25','0.20'),('nan','0.30'),
                              ('0','0.20'),('0.40','0.40')]:
            args=parser.parse_args(['robot-client','--robot-id','A','--host','localhost',
                                   '--map','unused','--journal','unused.sqlite3','--enable-motors',
                                   '--vendor-dir','unused','--drive-output',drive,'--max-output',maximum])
            with self.subTest(drive=drive,maximum=maximum):
                with patch('pi_car.common.grid.OccupancyGrid.load') as load:
                    with self.assertRaises(ValueError):
                        await run(args)
                    load.assert_not_called()

    async def test_one_missing_camera_detection_stops_then_resumes_only_on_fresh_pose(self):
        motor=RecordingMotor()
        agent=RobotAgent('A',mock_grid(),motor=motor,drive_output=.25)
        try:
            agent.connect()
            await agent.handle(agent.message('POSE',valid=True,x=1.,y=1.,theta=0.,timestamp=time.time()))
            await agent.handle(agent.message('GOAL',command_id='temporary-loss',goal=dict(x=1.5,y=1.)))
            await agent.planning
            agent.front_range=RangeReading(1.,0.,time.monotonic())
            agent.tick()
            self.assertNotEqual(motor.command,MotorCommand())
            await agent.handle(agent.message('POSE',valid=False,x=0.,y=0.,theta=0.,timestamp=time.time()))
            self.assertEqual(motor.command,MotorCommand())
            agent.tick()
            self.assertEqual(agent.active,'temporary-loss')
            await agent.handle(agent.message('POSE',valid=True,x=1.02,y=1.,theta=0.,timestamp=time.time()))
            agent.tick()
            self.assertNotEqual(motor.command,MotorCommand())
            await agent.handle(agent.message('POSE',valid=False,x=0.,y=0.,theta=0.,timestamp=time.time()))
            agent.pose_loss_since-=1.
            agent.tick()
            self.assertIsNone(agent.active)
            self.assertEqual(motor.command,MotorCommand())
            self.assertEqual(agent.journal.get('temporary-loss')[1]['type'],'ERROR')
            await agent.handle(agent.message('POSE',valid=True,x=1.02,y=1.,theta=0.,timestamp=time.time()))
            agent.tick()
            self.assertEqual(motor.command,MotorCommand())
        finally:
            await agent.close()


if __name__=='__main__':
    unittest.main()
