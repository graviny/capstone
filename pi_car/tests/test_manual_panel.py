import asyncio
import math
import tempfile
import time
from pathlib import Path
from types import SimpleNamespace
import unittest
import sys
from types import ModuleType
from unittest.mock import patch
import matplotlib
import numpy as np
matplotlib.use('Agg')

from pi_car.mock import MockFleet, mock_grid
from pi_car.pc.panel import PanelModel,FleetPanel,camera_floor_goal
from pi_car.pc.camera import RGBDFrame
from pi_car.pc.calibration import Calibration
from pi_car.common.geometry import Intrinsics
from pi_car.pc.commands import CommandInterface
from pi_car.transport.commands import CommandTCPServer
from pi_car.vehicle.hardware.live import FreshUltrasonic,AdeeptHardware


async def until(predicate, timeout=5.):
    deadline=time.monotonic()+timeout
    while not predicate():
        if time.monotonic()>deadline:raise AssertionError('Timeout')
        await asyncio.sleep(.01)


class PanelIntegrationTests(unittest.IsolatedAsyncioTestCase):
    async def test_click_selected_robot_through_real_api_and_stop(self):
        world=await MockFleet().start()
        gateway=await CommandTCPServer(CommandInterface(world.fleet).handle,port=0).start()
        panel=FleetPanel(world.grid,port=gateway.port)
        try:
            await until(lambda:all(a.connected and a.fresh_pose() for a in world.agents.values()))
            # Exercise asynchronous INFO/STATUS updates and GUI event → real TCP.
            async def ready():
                while not panel.model.states:
                    panel.refresh();await asyncio.sleep(.02)
            await asyncio.wait_for(ready(),3.)
            panel._select('B')
            # Real MouseEvent, including NumPy coordinates from the transform.
            from matplotlib.backend_bases import MouseEvent
            panel.fig.canvas.draw()
            # Rendering or a slow UI poll can leave an older display snapshot.
            # Clicking must use the live server pose, not this stale copy.
            panel.model.states['B']['pose']['timestamp'] -= 1.
            u,v=panel.ax.transData.transform((1.5,2.))
            panel._click(MouseEvent('button_press_event',panel.fig.canvas,u,v,button=1))
            await until(lambda:world.fleet.robots['B'].command_id is not None)
            command=world.fleet.robots['B'].command_id
            result=await world.fleet.wait_result(command,5.)
            self.assertEqual(result['type'],'ARRIVED')
            self.assertIsNone(world.fleet.robots['A'].command_id)
            panel.refresh()
            await until(lambda:panel.command_pending[1].done() if panel.command_pending else True)
            panel.refresh()
            # Query polling cannot monopolize the UI command lane.
            self.assertIsNotNone(panel.pending)
            panel._stop(None)
            await until(lambda:world.fleet.robots['B'].command_id!=command)
            stop = world.fleet.robots['B'].command_id
            self.assertEqual((await world.fleet.wait_result(stop,2.))['type'],'STOPPED')
            panel.refresh()
            await until(lambda:panel.command_pending[1].done() if panel.command_pending else True)
            panel.refresh()
            world.visible['B'] = False
            await until(lambda:world.fleet.robots['B'].pose is None)
            panel._click(MouseEvent('button_press_event',panel.fig.canvas,u,v,button=1))
            await until(lambda:panel.command_pending[1].done())
            panel.refresh()
            self.assertIn('no fresh valid pose', panel.notice.get_text())
            self.assertEqual(world.fleet.robots['B'].command_id, stop)
        finally:
            panel.close();panel.plt.close(panel.fig)
            await gateway.close();await world.close()


class PanelModelTests(unittest.TestCase):
    def test_camera_click_projects_floor_and_rejects_obstacle_or_stale_frame(self):
        intr=Intrinsics(100,100,100.,100.,0.,0.)
        depths=np.full((100,100),2.,dtype=np.float32)
        frame=RGBDFrame(np.zeros((100,100,3),dtype=np.uint8),depths,intr,
                        time.time(),'test-camera')
        calibration=Calibration([[1.,0.,0.,0.],[0.,-1.,0.,3.],
                                 [0.,0.,-1.,2.],[0.,0.,0.,1.]],
                                'test-floor','test-camera',vars(intr),0.)
        self.assertEqual(camera_floor_goal(frame,calibration,75,50),(1.5,2.))
        frame.depth_m[48:53,73:78]=1.5
        with self.assertRaisesRegex(ValueError,'floor'):
            camera_floor_goal(frame,calibration,75,50)
        frame.timestamp-=2
        with self.assertRaisesRegex(ValueError,'stale'):
            camera_floor_goal(frame,calibration,75,50)

    def test_numpy_mouse_coordinates_are_normalized_without_accepting_booleans(self):
        model=self.model()
        for x,y in [(np.float64(1.5),np.float64(1.)),(np.float32(1.5),np.int64(1))]:
            request=model.move_request('A',x,y)
            self.assertIs(type(request['goal']['x']),float)
            self.assertIs(type(request['goal']['y']),float)
        for x in [True,np.bool_(True),'1.5',np.float64('nan')]:
            with self.assertRaises(ValueError):model.move_request('A',x,1.)

    def model(self):
        model=PanelModel(mock_grid())
        model.set_info(dict(frame_id='mock-floor',map_version='mock-v1',clearance_m=.2,robot_ids=['A','B']))
        model.set_states({'A':dict(connected=True,pose=dict(
            valid=True,frame_id='mock-floor',timestamp=time.time(),x=1.,y=1.))})
        return model

    def test_click_goal_contract_and_no_automatic_goals(self):
        model=self.model()
        request=model.move_request('A',1.5,1.)
        self.assertEqual(request['type'],'MOVE')
        self.assertEqual(request['robot_id'],'A')
        self.assertEqual(request['frame_id'],'mock-floor')
        self.assertNotEqual(request['command_id'],model.move_request('A',1.5,1.)['command_id'])
        self.assertFalse(hasattr(model,'path'))  # panel does not plan on the PC

    def test_invalid_space_pose_and_map_rejected(self):
        model=self.model()
        for xy in [(-1.,1.),(0.,0.),(math.nan,1.)]:
            with self.assertRaises(ValueError):model.move_request('A',*xy)
        with self.assertRaises(ValueError):model.move_request('B',1.5,2.)
        model.states['A']['pose']['timestamp']-=1
        with self.assertRaises(ValueError):model.move_request('A',1.5,1.)
        with self.assertRaises(ValueError):model.set_info(dict(frame_id='old',map_version='mock-v1',clearance_m=.2,robot_ids=['A']))

    def test_turn_uses_signed_degrees_and_checks_current_clearance(self):
        model = self.model()
        self.assertEqual(model.turn_request('A', 25)['degrees'], 25)
        self.assertEqual(model.turn_request('A', -25)['degrees'], -25)
        for degrees in (0, True, 181, math.nan):
            with self.assertRaises(ValueError):
                model.turn_request('A', degrees)
        model.states['A']['pose']['x'] = 0.
        with self.assertRaises(ValueError):
            model.turn_request('A', 25)


class FreshSensorTests(unittest.TestCase):
    def test_gpiozero_mock_echo_updates_and_saturation_invalidates(self):
        try:
            from gpiozero import Device
            from gpiozero.pins.mock import MockFactory,MockTriggerPin
        except ImportError:self.skipTest('Optional gpiozero not installed')
        factory=MockFactory()
        echo=factory.pin(24)
        trigger=factory.pin(23,pin_class=MockTriggerPin)
        trigger.echo_pin,trigger.echo_time=echo,.004
        with patch.object(Device,'pin_factory',factory):
            sensor=FreshUltrasonic()
            try:
                deadline=time.monotonic()+1.
                while sensor.read() is None and time.monotonic()<deadline:time.sleep(.01)
                reading=sensor.read()
                self.assertIsNotNone(reading)
                self.assertTrue(.4 < reading.distance_m < 1.5)
                trigger.echo_time=.04
                deadline=time.monotonic()+1.
                while sensor.read() is not None and time.monotonic()<deadline:time.sleep(.01)
                self.assertIsNone(sensor.read())
            finally:sensor.close();factory.close()

    def test_hardware_bootstrap_centers_servo_and_cleans_up_without_real_devices(self):
        source='''
from types import SimpleNamespace
calls=[]
M1_Direction=M2_Direction=M3_Direction=M4_Direction=-1
def setup():
 global pwm_motor,motor1,motor2,motor3,motor4
 motor1,motor2,motor3,motor4=(SimpleNamespace(throttle=0.) for _ in range(4))
 pwm_motor=SimpleNamespace(channels=list(range(16)),deinit=lambda:calls.append('deinit'))
 calls.append('setup')
def Motor(*args):calls.append(('Motor',args))
def motorStop():calls.append('stop')
def destroy():pass
'''
        servo_module=ModuleType('adafruit_motor.servo')
        class Servo:
            def __init__(self,*args,**kwargs):self.angle=None
        servo_module.Servo=Servo
        sensor=SimpleNamespace(read=lambda:None,close=lambda:None)
        with tempfile.TemporaryDirectory() as directory:
            Path(directory,'Move.py').write_text(source)
            with patch.dict(sys.modules,{'adafruit_motor':ModuleType('adafruit_motor'),
                                         'adafruit_motor.servo':servo_module}), \
                 patch('pi_car.vehicle.hardware.live.FreshUltrasonic',return_value=sensor):
                hardware=AdeeptHardware(directory,max_output=.15)
                with hardware:
                    self.assertTrue(hardware.motor.enabled)
                    self.assertEqual(hardware.servo.angle,90.)
                    self.assertEqual(hardware.move.calls,['setup','stop'])
                self.assertIn('deinit',hardware.move.calls)
                self.assertIsNone(hardware.servo.angle)

    def test_only_actual_measurement_updates_timestamp_and_no_echo_invalidates(self):
        class FakeSensor:
            value=.25
            def __init__(self,**kwargs):self.kwargs=kwargs
            def _read(self):return self.value
            def close(self):pass
        sensor=FreshUltrasonic(sensor_class=FakeSensor)
        try:
            self.assertIsNone(sensor.read())
            sensor.sensor._read()
            first=sensor.read()
            self.assertAlmostEqual(first.distance_m,.5)
            self.assertIs(sensor.read(),first)
            self.assertEqual(sensor.read().received_monotonic,first.received_monotonic)
            sensor.sensor.value=None;sensor.sensor._read()
            self.assertIsNone(sensor.read())
            sensor.sensor.value=1.;sensor.sensor._read()
            self.assertIsNone(sensor.read())
        finally:sensor.close()

    def test_one_clipped_echo_preserves_original_age_and_two_invalidate(self):
        class FakeSensor:
            def __init__(self, **kwargs):
                pass

            def close(self):
                pass

        sensor = FreshUltrasonic(sensor_class=FakeSensor)
        try:
            with patch('pi_car.vehicle.hardware.live.time.monotonic', return_value=5.):
                sensor.sample(.55)
            last_measured = sensor.read()
            sensor.sample(1.)
            self.assertIs(sensor.read(), last_measured)
            self.assertTrue(sensor.read().usable(5.150))
            self.assertFalse(sensor.read().usable(5.201))
            sensor.sample(1.)
            self.assertIsNone(sensor.read())

            # A close obstacle replaces the previous clear distance immediately.
            sensor.sample(.55)
            sensor.sample(1.)
            sensor.sample(.15)
            self.assertAlmostEqual(sensor.read().distance_m, .30)
            sensor.sample(None)
            self.assertIsNone(sensor.read())
        finally:
            sensor.close()

    def test_hardware_options_fail_before_importing_vendor(self):
        with tempfile.TemporaryDirectory() as directory:
            Path(directory,'Move.py').write_text('raise AssertionError("must not import")')
            for kwargs in [dict(max_output=1.),dict(servo_angle=math.nan)]:
                with self.assertRaises(ValueError):AdeeptHardware(directory,**kwargs)

    def test_fresh_no_echo_is_clear_only_in_opt_in_reader(self):
        class FakeSensor:
            def __init__(self, **kwargs):pass
            def close(self):pass
        sensor=FreshUltrasonic(sensor_class=FakeSensor)
        try:
            with patch('pi_car.vehicle.hardware.live.time.monotonic',return_value=10.):
                sensor.sample(1.)
                sensor.sample(1.)
            self.assertIsNone(sensor.read())
            clear=sensor.read_allow_clear()
            self.assertEqual(clear.distance_m,2.)
            self.assertTrue(clear.usable(10.15))
            self.assertFalse(clear.usable(10.25))
            with patch('pi_car.vehicle.hardware.live.time.monotonic',return_value=11.):
                sensor.sample(.15)
            self.assertAlmostEqual(sensor.read_allow_clear().distance_m,.3)
            sensor.sample(None)
            self.assertIsNone(sensor.read_allow_clear())
        finally:sensor.close()

    def test_real_mode_rejects_fake_ranges_before_loading_any_hardware(self):
        from pi_car.fleet_cli import run
        args=SimpleNamespace(command='robot-client',mock_range=1.,enable_motors=True,vendor_dir='/unused')
        with self.assertRaises(ValueError):asyncio.run(run(args))


if __name__=='__main__':unittest.main()
