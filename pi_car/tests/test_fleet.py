import asyncio
import json
import math
from pathlib import Path
import tempfile
import time
import unittest
from unittest.mock import patch

from pi_car.common.messages import MotorCommand, Pose
from pi_car.mock import MockFleet, mock_grid
from pi_car.pc.commands import CommandInterface
from pi_car.transport.commands import CommandTCPServer, send_command
from pi_car.transport.protocol import decode_message, encode_message, make_message
from pi_car.transport.tcp import JsonConnection
from pi_car.vehicle.agent import RobotAgent
from pi_car.vehicle.hardware import DisabledMotor
from pi_car.vehicle.hardware.watchdog import WatchdogMotor


async def until(predicate, timeout=4.):
    deadline = time.monotonic()+timeout
    while not predicate():
        if time.monotonic() > deadline:
            raise AssertionError("Timed out waiting for fleet state")
        await asyncio.sleep(.01)


class FleetIntegrationTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.world = await MockFleet().start()
        self.fleet = self.world.fleet
        self.gateway = await CommandTCPServer(CommandInterface(self.fleet).handle,port=0).start()
        await until(lambda: all(a.connected and a.fresh_pose() for a in self.world.agents.values()))

    async def asyncTearDown(self):
        await self.gateway.close()
        await self.world.close()

    async def move(self, robot, x, y, command):
        response = await send_command("127.0.0.1",self.gateway.port,
                                     dict(type="MOVE",robot_id=robot,command_id=command,goal=dict(x=x,y=y)))
        self.assertTrue(response['ok'],response)
        return response['result']

    async def test_two_connections_only_target_moves_arrives_and_waits_for_next_command(self):
        self.assertEqual(set(self.fleet.snapshot()),{"A","B"})
        b_initial = self.world.positions['B'][:]
        await self.move("A",1.5,1.,"one")
        result = await self.fleet.wait_result("one",5.)
        self.assertEqual(result['type'],"ARRIVED")
        self.assertIsNone(self.world.agents['A'].active)
        await asyncio.sleep(.2)
        self.assertEqual(self.world.motors['A'].last_command,MotorCommand())
        self.assertEqual(self.world.positions['B'],b_initial)
        self.assertIsNone(self.fleet.robots['B'].command_id)
        self.assertEqual(len(self.fleet.journal.events()),1)  # no automatic next target
        await self.move("A",1.8,1.,"two")
        self.assertEqual((await self.fleet.wait_result("two",5.))['type'],"ARRIVED")

    async def test_independent_poses_and_concurrent_commands(self):
        self.assertNotEqual(self.fleet.robots['A'].pose.y,self.fleet.robots['B'].pose.y)
        await self.move("A",1.6,1.,"a")
        await self.move("B",1.6,2.,"b")
        a,b = await asyncio.gather(self.fleet.wait_result("a",5.),self.fleet.wait_result("b",5.))
        self.assertEqual([a['type'],b['type']],["ARRIVED","ARRIVED"])

    async def test_new_goal_cancels_old_goal_and_replans_on_pi(self):
        await self.move("A",2.5,1.,"old")
        await until(lambda: self.fleet.command_result('old')['type']=="MOVING")
        before = self.world.agents['A'].generation
        await self.move("A",1.4,1.,"replacement")
        self.assertEqual((await self.fleet.wait_result('old',2.))['type'],"CANCELLED")
        self.assertEqual((await self.fleet.wait_result('replacement',5.))['type'],"ARRIVED")
        self.assertGreater(self.world.agents['A'].generation,before)

    async def test_stop_cancels_only_target(self):
        await self.move("A",2.5,1.,"move")
        await until(lambda:self.fleet.command_result('move')['type']=="MOVING")
        await self.fleet.stop_robot("A","stop")
        self.assertEqual((await self.fleet.wait_result('stop',2.))['type'],"STOPPED")
        self.assertEqual(self.fleet.command_result('move')['type'],"CANCELLED")
        self.assertEqual(self.world.motors['A'].last_command,MotorCommand())
        self.assertIsNone(self.fleet.robots['B'].command_id)

    async def test_duplicate_commands_results_and_conflicting_ids(self):
        await self.move("A",1.5,1.,"same")
        await until(lambda:self.fleet.command_result('same')['type']=="MOVING")
        generation = self.world.agents['A'].generation
        await self.move("A",1.5,1.,"same")
        self.assertEqual(self.world.agents['A'].generation,generation)
        with self.assertRaises(ValueError):
            await self.fleet.dispatch_goal("B",1.5,2.,"same")
        arrived = await self.fleet.wait_result('same',5.)
        self.fleet.status_received(arrived)
        self.fleet.status_received(self.fleet.message("MOVING","A",command_id="same"))
        self.assertEqual(self.fleet.command_result('same'),arrived)
        # Deliver duplicate directly to Pi, bypassing PC deduplication.
        await self.fleet.robots['A'].sender.send(self.fleet.message("GOAL","A",command_id="same",
                                                                 goal=dict(x=1.5,y=1.)))
        await asyncio.sleep(.1)
        self.assertIsNone(self.world.agents['A'].active)
        self.assertEqual(self.world.motors['A'].last_command,MotorCommand())

    async def test_disconnect_reconnect_does_not_resume_and_other_robot_unaffected(self):
        await self.move("A",2.5,1.,"a")
        await self.move("B",1.7,2.,"b")
        await until(lambda:self.fleet.command_result('a')['type']=="MOVING")
        await self.fleet.robots['A'].sender.close()
        self.assertEqual((await self.fleet.wait_result('a',2.))['type'],"ERROR")
        await until(lambda:self.world.agents['A'].connected and self.world.agents['A'].fresh_pose())
        await asyncio.sleep(.15)
        self.assertIsNone(self.world.agents['A'].active)
        self.assertEqual(self.world.motors['A'].last_command,MotorCommand())
        self.assertEqual((await self.fleet.wait_result('b',5.))['type'],"ARRIVED")
        await self.move("A",1.5,1.,"new-after-reconnect")
        self.assertEqual((await self.fleet.wait_result('new-after-reconnect',5.))['type'],"ARRIVED")

    async def test_invalid_goals_ids_versions_and_unknown_robot(self):
        for x,y in ((-1.,1.),(0.,0.),(math.nan,1.),(True,1.)):
            with self.assertRaises(ValueError):
                await self.fleet.dispatch_goal('A',x,y,'invalid')
        with self.assertRaises(ValueError):
            await self.fleet.dispatch_goal('A',1.5,1.,'')
        with self.assertRaises(ValueError):
            await self.fleet.dispatch_goal('Z',1.5,1.,'unknown')
        with self.assertRaises(ValueError):
            await self.fleet.dispatch_goal('A',1.5,1.,'frame',frame_id='wrong')
        with self.assertRaises(ValueError):
            await self.fleet.dispatch_goal('A',1.5,1.,'version',map_version='wrong')
        self.assertEqual(self.world.motors['A'].last_command,MotorCommand())

    async def test_invalid_replacement_stops_running_command(self):
        await self.move('A',2.5,1.,'running')
        await until(lambda:self.fleet.command_result('running')['type']=='MOVING')
        with self.assertRaises(ValueError):
            await self.fleet.dispatch_goal('A',-1.,1.,'bad-goal')
        self.assertEqual((await self.fleet.wait_result('running',2.))['type'],'CANCELLED')
        await until(lambda:self.world.motors['A'].last_command==MotorCommand())

    async def test_tag_loss_only_stops_that_robot_and_requires_new_command(self):
        await self.move('A',2.5,1.,'a')
        await self.move('B',1.5,2.,'b')
        await until(lambda:self.fleet.command_result('a')['type']=='MOVING')
        self.world.visible['A'] = False
        self.assertEqual((await self.fleet.wait_result('a',2.))['type'],'ERROR')
        self.world.visible['A'] = True
        await asyncio.sleep(.1)
        self.assertIsNone(self.world.agents['A'].active)
        self.assertEqual((await self.fleet.wait_result('b',5.))['type'],'ARRIVED')

    async def test_obstacle_is_latched_blocked_until_new_command(self):
        await self.move('A',2.5,1.,'obstacle')
        await until(lambda:self.fleet.command_result('obstacle')['type']=='MOVING')
        self.world.range_m['A'] = .1
        self.assertEqual((await self.fleet.wait_result('obstacle',2.))['type'],'BLOCKED')
        self.world.range_m['A'] = 1.
        await asyncio.sleep(.15)
        self.assertEqual(self.world.motors['A'].last_command,MotorCommand())

    async def test_slow_planning_is_invalidated_by_stop(self):
        from pi_car.vehicle.planning import plan as real_plan
        def slow(*args):
            time.sleep(.15)
            return real_plan(*args)
        with patch('pi_car.vehicle.agent.plan',side_effect=slow):
            await self.move('A',2.5,1.,'slow')
            await until(lambda:self.world.agents['A'].active=='slow')
            await self.fleet.stop_robot('A','stop-slow')
            self.assertEqual((await self.fleet.wait_result('stop-slow',2.))['type'],'STOPPED')
            await asyncio.sleep(.25)
        self.assertIsNone(self.world.agents['A'].active)
        self.assertEqual(self.world.motors['A'].last_command,MotorCommand())

    async def test_wire_map_mismatch_stops_target(self):
        await self.move('A',2.5,1.,'map-mismatch')
        await until(lambda:self.fleet.command_result('map-mismatch')['type']=='MOVING')
        bad=self.fleet.message('POSE','A',valid=True,x=1.,y=1.,theta=0.,timestamp=time.time())
        bad['map_version']='wrong'
        await self.fleet.robots['A'].sender.send(bad)
        self.assertEqual((await self.fleet.wait_result('map-mismatch',2.))['type'],'ERROR')
        await until(lambda:self.world.motors['A'].last_command==MotorCommand())

    async def test_duplicate_registration_and_wrong_map_are_rejected(self):
        from pi_car.transport.tcp import open_connection
        for changes in ({},dict(robot_id='Z'),dict(map_version='other')):
            connection=await open_connection('127.0.0.1',self.world.server.port,.5)
            hello=self.fleet.message('HELLO','A',clearance_m=.2)
            hello.update(changes)
            await connection.send(hello)
            with self.assertRaises((ConnectionError,OSError)):
                await connection.receive()
            await connection.close()
        self.assertTrue(self.fleet.robots['A'].connected)
        self.assertTrue(self.fleet.robots['B'].connected)

    async def test_pi_planning_error_is_reported_without_motion(self):
        with patch('pi_car.vehicle.agent.plan',side_effect=ValueError('No reachable path')):
            await self.move('A',1.5,1.,'no-path')
            result=await self.fleet.wait_result('no-path',2.)
        self.assertEqual(result['type'],'ERROR')
        self.assertIn('PLANNING_FAILED',result['reason'])
        self.assertEqual(self.world.motors['A'].last_command,MotorCommand())


class TransportTests(unittest.IsolatedAsyncioTestCase):
    async def test_fragmented_coalesced_and_truncated_frames(self):
        message=make_message('STOP','A','floor','v1',command_id='stop1')
        raw=encode_message(message)
        reader=asyncio.StreamReader(limit=16384)
        conn=JsonConnection(reader,None,.2)
        task=asyncio.create_task(conn.receive())
        for piece in (raw[:5],raw[5:11],raw[11:]):
            reader.feed_data(piece)
            await asyncio.sleep(0)
        self.assertEqual(await task,message)
        reader.feed_data(raw+raw)
        self.assertEqual(await conn.receive(),message)
        self.assertEqual(await conn.receive(),message)
        reader.feed_data(raw[:-1]);reader.feed_eof()
        with self.assertRaises(ValueError):
            await conn.receive()

    async def test_pose_timeout_even_when_tcp_heartbeat_is_alive(self):
        agent=RobotAgent('A',mock_grid())
        try:
            agent.connect()
            await agent.handle(agent.message('POSE',valid=True,x=1.,y=1.,theta=0.,timestamp=time.time()))
            await agent.handle(agent.message('GOAL',command_id='stale',goal=dict(x=2.,y=1.)))
            await until(lambda:bool(agent.runtime.path))
            agent.pose_received=time.monotonic()-1.
            agent.tick()
            self.assertEqual(agent.journal.get('stale')[1]['type'],'ERROR')
            self.assertIsNone(agent.active)
        finally:await agent.close()

    async def test_durable_duplicate_after_process_restart(self):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory,'pi.sqlite3')
            agent=RobotAgent('A',mock_grid(),journal_path=path)
            agent.connect()
            request=agent.message('GOAL',command_id='persistent',goal=dict(x=2.,y=1.))
            await agent.handle(request)  # no pose -> terminal ERROR
            await agent.close()
            restarted=RobotAgent('A',mock_grid(),journal_path=path)
            try:
                restarted.connect()
                await restarted.handle(request)
                self.assertIsNone(restarted.active)
                self.assertEqual(restarted.journal.get('persistent')[1]['type'],'ERROR')
            finally:await restarted.close()

    async def test_partial_oversized_input_and_receive_timeout(self):
        reader=asyncio.StreamReader(limit=32)
        connection=JsonConnection(reader,None,.02)
        reader.feed_data(b'x'*40)
        with self.assertRaises(ValueError):await connection.receive()
        empty=JsonConnection(asyncio.StreamReader(),None,.02)
        with self.assertRaises(asyncio.TimeoutError):await empty.receive()

    async def test_unknown_and_occupied_goals_rejected_by_pi(self):
        grid=mock_grid();grid.cells[10,20]=-1;grid.cells[20,20]=1
        agent=RobotAgent('A',grid,clearance_m=0.)
        try:
            agent.connect()
            await agent.handle(agent.message('POSE',valid=True,x=1.,y=1.,theta=0.,timestamp=time.time()))
            for command_id,y in [('unknown',1.05),('occupied',2.05)]:
                await agent.handle(agent.message('GOAL',command_id=command_id,goal=dict(x=2.05,y=y)))
                self.assertEqual(agent.journal.get(command_id)[1]['reason'],'GOAL_NOT_FREE')
                self.assertIsNone(agent.active)
        finally:await agent.close()


class ProtocolAndWatchdogTests(unittest.TestCase):
    def test_schema_rejects_nonfinite_boolean_bad_ids_and_duplicate_keys(self):
        for raw in (b'{"schema_version":1,"schema_version":1}\n',b'{"x":NaN}\n',b'[]\n'):
            with self.assertRaises(ValueError):decode_message(raw)
        with self.assertRaises(ValueError):
            make_message('GOAL','A','floor','v1',command_id='bad id',goal=dict(x=1,y=2))
        with self.assertRaises(ValueError):
            make_message('GOAL','A','floor','v1',command_id='id',goal=dict(x=True,y=2))

    def test_watchdog_stops_without_control_loop_and_stays_latched(self):
        driver=DisabledMotor();guard=WatchdogMotor(driver,.05)
        try:
            guard.arm();guard.apply(MotorCommand(.2,.2))
            time.sleep(.1)
            self.assertEqual(driver.last_command,MotorCommand())
            with self.assertRaises(RuntimeError):guard.apply(MotorCommand(.2,.2))
        finally:guard.close()


if __name__=='__main__':unittest.main()
