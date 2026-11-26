"""Geometry, obstacle replan and watchdog checks without physical devices."""
import csv
import math
import socket
from tempfile import TemporaryDirectory
from pathlib import Path
import unittest
from unittest.mock import Mock, patch

import cv2
import numpy as np

from navigation_core import FloorCalibration, OccupancyGrid, Navigator, Pose
from navigation_io import MotionWorker, NativeMiio
from navigation_sim import SimulatedCamera
from navigation_app import Session


def floor_points(width=3,height=2):
    x,y = np.meshgrid(np.arange(.0125,width,.025),np.arange(.0125,height,.025))
    return np.column_stack([x.ravel(),y.ravel(),np.zeros(x.size)])


class GeometryTests(unittest.TestCase):
    def test_camera_height_entered_as_ipad_height_gets_specific_error(self):
        import copy
        camera = SimulatedCamera()
        data = copy.deepcopy(camera.calibration.data)
        data["marker_height_m"] = camera.camera[2]
        with self.assertRaisesRegex(ValueError,"아이패드 높이 2.50m.*카메라 높이 2.50m"):
            FloorCalibration(data)

    def test_invalid_rotation_is_reported_separately_from_height(self):
        import copy
        data = copy.deepcopy(SimulatedCamera().calibration.data)
        data["world_to_camera_R"][0][0] = 2
        with self.assertRaisesRegex(ValueError,"회전・이동 행렬"):
            FloorCalibration(data)

    def test_rendered_camera_to_goal_does_not_stall_at_clearance_corner(self):
        camera = SimulatedCamera()
        session = Session()
        nav = Navigator()
        nav.set_goal((2.3,1.45))
        with patch("navigation_sim.time.monotonic") as clock:
            for step in range(650):
                clock.return_value = camera.last+.1
                packet,_ = camera.latest()
                if session.calibration is None:
                    session.calibrate(camera.calibration,packet)
                session.process(packet)
                self.assertTrue(session.healthy)
                if step==1:
                    nav.running = True
                camera.velocity,camera.rotation = nav.step(session.pose,session.grid,packet["stamp"],True)
                # Compare with the fixture's true obstacle, independently of
                # the depth grid and its robot masking.
                dx = max(1.35-camera.pose.x,0,camera.pose.x-1.65)
                dy = max(.8-camera.pose.y,0,camera.pose.y-1.2)
                self.assertGreater(math.hypot(dx,dy),.18+.15-.025)
                if nav.message=="목표점 도착":
                    break
        self.assertEqual(nav.message,"목표점 도착")
        self.assertLessEqual(math.dist((camera.pose.x,camera.pose.y),nav.goal),.13)

    def test_metric_calibration_recovers_camera_and_marker_height(self):
        camera = SimulatedCamera()
        cal = FloorCalibration.fit(camera.calibration.data["reference_pixels"],3,2,.12,
                                    camera.ray,camera.metadata)
        np.testing.assert_allclose(cal.camera,camera.camera,atol=1e-5)
        expected = np.array([.8,.7,.12])
        front = expected+np.array([.1,0,0])
        image = camera.project([expected,front])
        pose = cal.marker_pose(camera.ray(image[0]),camera.ray(image[1]),10)
        self.assertAlmostEqual(pose.x,.8,places=5)
        self.assertAlmostEqual(pose.y,.7,places=5)
        self.assertAlmostEqual(pose.yaw,0,places=5)
        wrong = cal.intersect(camera.ray(image[0]),0)
        self.assertGreater(np.linalg.norm(wrong[:2]-expected[:2]),.03)

    def test_camera_tilt_is_calibrated(self):
        camera = SimulatedCamera()
        rotation = cv2.Rodrigues(np.array([.06,-.04,.02]))[0]
        camera.R = rotation@camera.R
        camera.t = -camera.R@camera.camera
        points = camera.project([[0,2,0],[3,2,0],[3,0,0],[0,0,0]])
        cal = FloorCalibration.fit(points,3,2,.12,camera.ray,camera.metadata)
        np.testing.assert_allclose(cal.camera,camera.camera,atol=2e-5)

    def test_changed_camera_or_profile_is_rejected(self):
        import copy
        camera = SimulatedCamera()
        for field in ("serial","fx"):
            metadata = copy.deepcopy(camera.metadata)
            if field=="serial":
                metadata["device"][field] = "OTHER"
            else:
                metadata["color_intrinsics"][field] += 5
            with self.assertRaises(ValueError):
                camera.calibration.validate_camera(metadata)

    def test_ipad_marker_detects_and_maps_without_using_surface_depth(self):
        camera = SimulatedCamera()
        packet,_ = camera.latest()
        session = Session()
        session.calibrate(camera.calibration,packet)
        # Reflective display can have no depth. Floor rays still locate marker.
        session.process(packet)
        self.assertIsNotNone(session.pose)
        self.assertAlmostEqual(session.pose.x,camera.pose.x,delta=.04)
        self.assertAlmostEqual(session.pose.y,camera.pose.y,delta=.04)
        self.assertTrue(session.healthy)

    def test_missing_marker_does_not_map_robots_body_as_an_obstacle(self):
        camera = SimulatedCamera()
        packet,_ = camera.latest()
        session = Session()
        session.calibrate(camera.calibration,packet)
        session.process(packet)
        self.assertTrue(session.healthy)
        obstacles = session.grid.obstacles.copy()

        # Keep the elevated depth returns from the robot and hide only its tag.
        missing = dict(packet,frame=np.full_like(packet["frame"],245),stamp=packet["stamp"]+.1)
        session.process(missing)
        self.assertFalse(session.healthy)
        np.testing.assert_array_equal(session.grid.obstacles,obstacles)

        camera.pose.x += .35
        relocated,_ = camera.latest()
        relocated["stamp"] = packet["stamp"]+.85
        session.process(relocated)
        self.assertTrue(session.healthy)
        self.assertTrue(session.grid.plan((session.pose.x,session.pose.y),(.55,.55)))
        x,y = session.grid.cell((1.5,1.0))
        self.assertTrue(session.grid.obstacles[y,x])

    def test_roi_recovers_marker_from_current_frame_when_full_detector_misses(self):
        camera = SimulatedCamera()
        packet,_ = camera.latest()
        session = Session()
        session.calibrate(camera.calibration,packet)
        session.process(packet)
        original = session.tracker.detector
        camera.pose.x += .03
        latest,_ = camera.latest()
        latest["stamp"] = packet["stamp"]+.1
        detector = Mock()
        detector.detectMarkers.side_effect = lambda image: (
            ((),None,()) if image.shape == latest["frame"].shape[:2] else original.detectMarkers(image))
        session.tracker.detector = detector
        raw = session.process(latest,annotate=False)
        self.assertIs(raw,latest["frame"])
        self.assertTrue(session.healthy)
        self.assertEqual(session.pose.stamp,latest["stamp"])
        self.assertAlmostEqual(session.pose.x,camera.pose.x,delta=.015)
        self.assertEqual(detector.detectMarkers.call_count,2)

    def test_interleaved_depth_samples_find_real_floor_without_filling_an_unobserved_patch(self):
        camera = SimulatedCamera()
        packet,_ = camera.latest()
        depth = np.zeros_like(packet["depth"])
        depth[1::3,2::3] = packet["depth"][1::3,2::3]
        depth[140:190,430:480] = 0
        packet["depth"] = depth
        session = Session()
        session.calibrate(camera.calibration,packet)
        session.process(packet)
        self.assertFalse(session.healthy)

        session.process(dict(packet,stamp=packet["stamp"]+.1))
        self.assertTrue(session.healthy)
        observed_x,observed_y = session.grid.cell((2.1,.5))
        self.assertTrue(session.grid.known[observed_y,observed_x])
        missing = camera.calibration.intersect(camera.ray((455,165)))
        missing_x,missing_y = session.grid.cell(missing[:2])
        self.assertFalse(session.grid.known[missing_y,missing_x])


class PlannerTests(unittest.TestCase):
    def grid(self):
        grid = OccupancyGrid(3,2)
        grid.update_points(floor_points(),Pose(.6,.5,0,0),0)
        return grid

    def test_small_heading_error_near_boundary_aligns_instead_of_replan_loop(self):
        grid = self.grid()
        pose = Pose(1.25,.36,-.2,10)
        nav = Navigator()
        nav.goal = (2.4,.36)
        nav.path = [(pose.x,pose.y),nav.goal]
        nav.running = True
        # At this heading a short forward move crosses the inflated border.
        # The route itself is valid, so another copy of that route cannot help.
        grid.segment_free = lambda a,b: b[1] >= .35
        speed,rotation = nav.step(pose,grid,10)
        self.assertEqual(speed,0)
        self.assertGreater(rotation,0)
        self.assertEqual(nav.replans,0)
        self.assertTrue(nav.running)

    def test_path_routes_around_inflated_obstacle(self):
        grid = self.grid()
        grid.add_rectangle((1.3,.7),(1.6,1.1))
        grid.update_points(floor_points(),Pose(.6,.5,0,0),.1)
        path = grid.plan((.6,.5),(2.4,1.4))
        self.assertGreater(len(path),2)
        self.assertTrue(all(grid.segment_free(a,b) for a,b in zip(path,path[1:])))

    def test_unknown_space_never_becomes_a_shortcut(self):
        grid = OccupancyGrid(3,2)
        self.assertEqual(grid.plan((.6,.5),(2.4,1.4)),[])
        grid.update_points(floor_points(),Pose(.6,.5,0,0),0)
        grid.update_points(np.empty((0,3)),None,1)
        self.assertEqual(grid.plan((.6,.5),(2.4,1.4)),[])

    def test_isolated_far_depth_needs_a_second_frame_and_clears_after_stable_floor(self):
        grid = self.grid()
        pose = Pose(.6,.5,0,0)
        floor = floor_points()
        route = ((pose.x,pose.y),(2.4,.5))
        noisy = np.vstack([floor, [1.775,.775,.08]])

        grid.update_points(noisy,pose,.1)
        self.assertTrue(grid.segment_free(*route))
        grid.update_points(noisy,pose,.1)
        self.assertTrue(grid.segment_free(*route))
        grid.update_points(floor,pose,.2)
        grid.update_points(noisy,pose,.3)
        self.assertFalse(grid.segment_free(*route))
        self.assertTrue(grid.depth_obstacle_blocks([route]))

        for now in (.4,.8,1.2,1.5):
            grid.update_points(floor,pose,now)
            self.assertFalse(grid.segment_free(*route))
        grid.update_points(floor,pose,1.7)
        self.assertTrue(grid.segment_free(*route))

    def test_near_isolated_depth_is_immediate_and_weak_unknown_cell_stays_blocked(self):
        grid = self.grid()
        pose = Pose(.6,.5,0,0)
        near = np.vstack([floor_points(),[.925,.525,.08]])
        grid.update_points(near,pose,.1)
        self.assertFalse(grid.segment_free((pose.x,pose.y),(1.5,.5)))
        self.assertTrue(grid.depth_obstacle_blocks([((pose.x,pose.y),(1.5,.5))]))

        unknown = OccupancyGrid(3,2)
        unknown.update_points([[1.775,.775,.08]],pose,.1)
        x,y = unknown.cell((1.775,.775))
        self.assertFalse(unknown.known[y,x])
        self.assertFalse(unknown.free((1.775,.775)))

    def test_far_boundary_cluster_needs_two_frames_and_isolated_point_needs_three(self):
        grid = self.grid()
        pose = Pose(.6,.5,0,0)
        floor = floor_points()
        lone = (2.525,.225,.08)
        clustered = (1.825,.225,.08)
        returns = np.vstack([floor,lone,clustered,[1.775,.225,.08],[1.875,.225,.08]])
        lx,ly = grid.cell(lone[:2])
        cx,cy = grid.cell(clustered[:2])
        grid.update_points(returns,pose,.10)
        self.assertFalse(grid.obstacles[cy,cx])
        grid.update_points(returns,pose,.20)
        self.assertTrue(grid.obstacles[cy,cx])
        self.assertFalse(grid.obstacles[ly,lx])
        grid.update_points(returns,pose,.30)
        self.assertTrue(grid.obstacles[ly,lx])

        near_grid = self.grid()
        near_grid.update_points(np.vstack([floor,lone]),Pose(2.5,.55,0,.1),.1)
        self.assertTrue(near_grid.obstacles[ly,lx])

    def test_blocked_goal_is_not_silently_moved(self):
        grid = self.grid()
        grid.add_rectangle((2.2,1.2),(2.6,1.6))
        grid.update_points(floor_points(),Pose(.6,.5,0,0),.1)
        self.assertEqual(grid.plan((.6,.5),(2.4,1.4)),[])

    def test_stop_reason_survives_frame_recovery_and_route_preview(self):
        grid = self.grid()
        nav = Navigator()
        nav.set_goal((2.4,.5))
        nav.running = True
        nav.step(None,grid,.1,healthy=False,measurement_issue="마커 인식 끊김")
        pose = Pose(.6,.5,0,.2)
        nav.step(pose,grid,.2)
        self.assertTrue(nav.path)
        self.assertFalse(nav.running)
        self.assertEqual(nav.last_stop_reason,"마커 인식 끊김")

    def test_missing_depth_on_a_route_reports_a_distinct_stop_reason(self):
        grid = self.grid()
        nav = Navigator()
        nav.set_goal((2.4,.5))
        nav.running = True
        nav.step(Pose(.6,.5,0,0),grid,0)
        floor = floor_points()
        hole = (abs(floor[:,0]-1.5)<.1) & (abs(floor[:,1]-.5)<.1)
        pose = Pose(.6,.5,0,.8)
        grid.update_points(floor[~hole],pose,.8)
        self.assertEqual(nav.step(pose,grid,.8),(0,0))
        self.assertIn("깊이 미측정 구역",nav.message)

    def test_new_obstacle_waits_for_stop_and_current_pose_before_replan(self):
        grid = self.grid()
        nav = Navigator()
        nav.set_goal((2.4,.5))
        nav.running = True
        nav.step(Pose(.6,.5,0,0),grid,0)
        nav.step(Pose(.6,.5,0,.5),grid,.5)
        before = nav.replans
        grid.add_rectangle((1.35,.35),(1.65,.65))
        grid.update_points(floor_points(),Pose(.6,.5,0,.6),.6)
        with patch.object(grid,"plan",wraps=grid.plan) as plan:
            self.assertEqual(nav.step(Pose(.6,.5,0,.6),grid,.6,stop_confirmed=False),(0.0,0))
            self.assertEqual(nav.path,[])
            self.assertEqual(nav.replans,before)
            for t in (.7,.8):
                nav.step(Pose(.6,.5,0,t),grid,t,stop_confirmed=False,verify_stop=True)
            nav.step(Pose(.6,.5,0,.9),grid,.9,verify_stop=True)
            nav.step(Pose(.64,.5,0,1.05),grid,1.05,verify_stop=True)
            plan.assert_not_called()
            self.assertEqual(nav.step(Pose(.64,.5,0,1.2),grid,1.2,verify_stop=True),(0,0))
            plan.assert_called_once_with((.64,.5),nav.goal)
        self.assertEqual(nav.replans,before+1)
        self.assertGreater(len(nav.path),2)
        self.assertNotEqual(nav.step(Pose(.64,.5,0,1.3),grid,1.3),(0,0))

    def test_replan_without_route_stays_zero_and_preserves_goal(self):
        grid = self.grid()
        nav = Navigator()
        goal = (2.4,.5)
        nav.set_goal(goal)
        nav.running = True
        nav.step(Pose(.6,.5,0,0),grid,0)
        grid.add_rectangle((1.3,0),(1.6,2))
        grid.update_points(floor_points(),Pose(.6,.5,0,.6),.6)
        self.assertEqual(nav.step(Pose(.6,.5,0,.6),grid,.6),(0,0))
        self.assertEqual(nav.step(Pose(.6,.5,0,.8),grid,.8),(0,0))
        self.assertEqual(nav.path,[])
        self.assertEqual(nav.goal,goal)
        self.assertEqual(nav.step(None,grid,1),(0,0))
        self.assertFalse(nav.running)

    def test_lost_pose_latches_stop(self):
        nav = Navigator()
        nav.set_goal((2,.5))
        nav.running = True
        self.assertEqual(nav.step(None,self.grid(),1),(0.0,0))
        self.assertFalse(nav.running)
        nav.step(Pose(.6,.5,0,2),self.grid(),2)
        self.assertFalse(nav.running)

    def test_feedback_reaches_target_in_software_simulation(self):
        grid = self.grid()
        grid.add_rectangle((1.35,.7),(1.65,1.1))
        nav = Navigator()
        nav.set_goal((2.3,1.45))
        nav.running = True
        pose = Pose(.6,.5,0,0)
        for i in range(1500):
            now = i*.1
            pose.stamp = now
            grid.update_points(floor_points(),pose,now)
            v,w = nav.step(pose,grid,now)
            pose.yaw += math.radians(w)*.1
            pose.x += math.cos(pose.yaw)*v*.1
            pose.y += math.sin(pose.yaw)*v*.1
            self.assertTrue(grid.free((pose.x,pose.y)))
            if not nav.running:
                break
        self.assertEqual(nav.message,"목표점 도착")
        self.assertLessEqual(math.dist((pose.x,pose.y),nav.goal),.12)


class WorkerTests(unittest.TestCase):
    def make(self):
        self.now = 0.0
        self.client = Mock()
        self.factory = Mock(return_value=self.client)
        return MotionWorker(self.factory,clock=lambda:self.now)

    def test_stale_measurement_does_not_even_open_robot_connection(self):
        worker = self.make()
        worker.submit(.08,0,0,True)
        self.now = 1
        worker.tick()
        self.factory.assert_not_called()

    def test_warmup_then_fresh_command_only(self):
        worker = self.make()
        worker.submit(.08,0,0,True)
        worker.tick()
        self.client.move.assert_not_called()
        self.now = 5.1
        worker.submit(.04,15,self.now,True)
        worker.tick()
        self.client.move.assert_called_once_with(.04,15)

    def test_move_rate_limit_and_queued_stop_precedes_the_next_move(self):
        worker = self.make()
        worker.submit(.08,0,0,True)
        worker.tick()
        self.now = 5.1
        worker.submit(.04,15,self.now,True)
        worker.tick()
        self.now = 5.2
        worker.submit(.05,0,self.now,True)
        worker.tick()
        self.client.move.assert_called_once_with(.04,15)

        worker.submit()
        worker.submit(.06,0,self.now,True)
        worker.tick()
        self.client.halt.assert_called_once()
        self.assertEqual(self.client.move.call_count,1)

    def test_missing_move_reply_checks_link_and_then_uses_latest_command(self):
        worker = self.make()
        worker.submit(.08,0,0,True)
        worker.tick()
        self.client.move.side_effect = [socket.timeout("timed out"),None]
        self.now = 5.1
        worker.submit(.04,15,self.now,True)
        with self.assertLogs("navigation_io",level="WARNING"):
            worker.tick()
        self.now = 5.45
        worker.submit(.06,0,self.now,True)
        worker.tick()
        self.client.confirm_manual_mode.assert_called_once()
        self.client.move.assert_called_once_with(.04,15)
        self.now = 5.5
        worker.submit(.07,0,self.now,True)
        worker.tick()
        self.client.move.assert_called_with(.07,0)
        self.client.halt.assert_not_called()
        self.assertFalse(worker.fault)

    def test_link_check_failure_halts_with_no_further_motion(self):
        worker = self.make()
        worker.submit(.08,0,0,True)
        worker.tick()
        self.client.move.side_effect = socket.timeout("timed out")
        self.client.confirm_manual_mode.side_effect = socket.timeout("timed out")
        self.now = 5.1
        worker.submit(.05,0,self.now,True)
        with self.assertLogs("navigation_io",level="WARNING"):
            worker.tick()
        self.now = 5.45
        worker.submit(.06,0,self.now,True)
        worker.tick()
        self.client.move.assert_called_once()
        self.client.halt.assert_called_once()
        self.assertIn("응답 누락 후 청소기 상태 조회",worker.fault)

    def test_camera_freeze_halts_independently_of_gui(self):
        worker = self.make()
        worker.submit(.08,0,0,True)
        worker.tick()
        self.now = .5
        worker.tick()
        self.client.halt.assert_called_once()
        self.client.move.assert_not_called()

    def test_stop_event_records_the_trigger_even_if_the_robot_already_replied(self):
        worker = self.make()
        with TemporaryDirectory() as folder:
            worker.event_log = Path(folder)/"drive_events.csv"
            worker.submit(.08,0,0,True)
            worker.tick()
            worker.submit(stop_reason="목표점 도착")
            worker.tick()
            self.assertTrue(worker.stop_confirmed())
            with worker.event_log.open(encoding="utf-8",newline="") as stream:
                rows = list(csv.DictReader(stream))
            self.assertEqual([row["event"] for row in rows],["stop"])
            self.assertEqual(rows[0]["detail"],"목표점 도착")

    def test_stop_confirmation_follows_halt_and_is_invalidated_by_new_motion(self):
        worker = self.make()
        worker.tick()
        self.assertTrue(worker.stop_confirmed())
        worker.submit(.08,0,0,True)
        worker.tick()
        worker.submit()
        self.assertFalse(worker.stop_confirmed())
        self.client.halt.side_effect = lambda: self.assertFalse(worker.stop_confirmed())
        worker.tick()
        self.assertTrue(worker.stop_confirmed())
        worker.submit(.08,0,0,True)
        self.assertFalse(worker.stop_confirmed())

    def test_failed_halt_is_not_acknowledged(self):
        worker = self.make()
        worker.submit(.08,0,0,True)
        worker.tick()
        worker.submit()
        self.client.halt.side_effect = OSError("device stopped replying")
        worker.tick()
        self.assertTrue(worker.fault)
        self.assertFalse(worker.stop_confirmed())

    def test_start_timeout_still_attempts_stop_and_latches_fault(self):
        worker = self.make()
        self.client.start.side_effect = OSError("network lost")
        worker.submit(.08,0,0,True)
        worker.tick()
        self.client.halt.assert_called_once()
        self.assertIn("수동 모드 진입 확인",worker.fault)
        self.client.start.reset_mock()
        worker.submit(.08,0,0,True)
        worker.tick()
        self.client.start.assert_not_called()

    def test_native_handshake_uses_bounded_timeout_and_no_retry(self):
        from miio.exceptions import DeviceException
        with TemporaryDirectory() as folder:
            client = NativeMiio("192.0.2.1","0"*32,sequence_file=Path(folder)/"seq.json")
            with patch("miio.miioprotocol.MiIOProtocol.discover",return_value=None) as discover:
                with self.assertRaises(DeviceException):
                    client.device._protocol.send_handshake()
            discover.assert_called_once_with("192.0.2.1",timeout=.4)
            self.assertEqual(client.device._protocol._timeout,.4)
            self.assertEqual(client.device.retry_count,0)

    def test_sequence_file_permission_error_keeps_stop_responses_and_real_timeout(self):
        with TemporaryDirectory() as folder:
            client = NativeMiio("192.0.2.1","0"*32,sequence_file=Path(folder)/"seq.json")
            with patch.object(Path,"replace",side_effect=PermissionError("sequence file busy")), \
                 patch.object(client.device,"manual_control",return_value=None), \
                 patch.object(client.device,"manual_stop",return_value=["ok"]), \
                 patch.object(client.device,"pause",return_value=["ok"]), \
                 self.assertLogs("navigation_io",level="WARNING"):
                client.halt()
                with self.assertRaisesRegex(socket.timeout,"device response timed out"):
                    client._call(Mock(side_effect=socket.timeout("device response timed out")))
            with (Path(folder)/"stop_command_timing.csv").open(encoding="utf-8",newline="") as stream:
                rows = list(csv.DictReader(stream))
            self.assertEqual([row["result"] for row in rows],["responded"]*3)
            self.assertFalse(client.sequence_temp.exists())

    def test_motion_sequence_saves_are_limited_and_stop_still_saves(self):
        with TemporaryDirectory() as folder:
            client = NativeMiio("192.0.2.1","0"*32,sequence_file=Path(folder)/"seq.json")
            with patch.object(client,"_save_sequence") as save, \
                 patch.object(client.device,"manual_control") as move, \
                 patch("navigation_io.time.monotonic",side_effect=[1.,1.3,1.6,2.1,2.2]):
                for _ in range(4):
                    client.move(.05,0)
                self.assertEqual(save.call_count,2)
                client._call(move,rotation=0,velocity=0.,duration=300)
            self.assertEqual(save.call_count,3)

    def test_preflight_retries_only_the_read_only_status_after_a_timeout(self):
        with TemporaryDirectory() as folder:
            client = NativeMiio("192.0.2.1","0"*32,sequence_file=Path(folder)/"seq.json")
            status = Mock(error_code=0,battery=50,state_code=3)
            with patch.object(client,"_call",side_effect=[socket.timeout("timed out"),status]) as call:
                self.assertIs(client.check(retry_on_timeout=True),status)
            self.assertEqual([entry.args for entry in call.call_args_list],
                             [(client.device.status,),(client.device.status,)])

    def test_socket_timeout_retries_only_one_stop_command_and_keeps_unverified_fault(self):
        from miio.exceptions import DeviceException
        with TemporaryDirectory() as folder:
            client = NativeMiio("192.0.2.1","0"*32,sequence_file=Path(folder)/"seq.json")
            failure = DeviceException("No response from the device")
            failure.__cause__ = socket.timeout("timed out")
            with patch.object(client,"_call",side_effect=[None,failure,["ok"],["ok"],failure]) as call:
                with self.assertRaisesRegex(RuntimeError,"app_rc_end"):
                    client.halt()
            self.assertEqual(call.call_count,5)
            self.assertEqual(call.call_args_list[2].args,(client.device.pause,))
            self.assertEqual(call.call_args_list[3].args,(client.device.manual_stop,))
            call.assert_called_with(client._require_stopped_status)

            with (Path(folder)/"stop_command_timing.csv").open(encoding="utf-8",newline="") as stream:
                rows = list(csv.DictReader(stream))
            self.assertEqual([row["command"] for row in rows],
                             ["app_rc_move_zero","app_rc_end","app_pause","app_rc_end","get_status_after_stop"])
            self.assertEqual([row["result"] for row in rows],
                             ["responded","timeout","responded","responded","timeout"])
            self.assertEqual(len({row["stop_started_at"] for row in rows}),1)
            self.assertIn("timed out",rows[1]["detail"])

    def test_zero_move_timeout_requires_idle_or_paused_after_both_stop_responses(self):
        for state_code,state in ((10,"Paused"),(7,"Manual mode")):
            with self.subTest(state=state),TemporaryDirectory() as folder:
                client = NativeMiio("192.0.2.1","0"*32,sequence_file=Path(folder)/"seq.json")
                status = Mock(state_code=state_code,state=state,error_code=0)
                with patch.object(client.device,"manual_control",side_effect=socket.timeout("timed out")), \
                     patch.object(client.device,"manual_stop",return_value=["ok"]), \
                     patch.object(client.device,"pause",return_value=["ok"]), \
                     patch.object(client.device,"status",return_value=status) as query:
                    if state_code == 10:
                        client.halt()
                    else:
                        with self.assertRaisesRegex(RuntimeError,"정지 상태 미확인"):
                            client.halt()
                self.assertEqual(query.call_count,1 if state_code == 10 else 2)
                with (Path(folder)/"stop_command_timing.csv").open(encoding="utf-8",newline="") as stream:
                    rows = list(csv.DictReader(stream))
                self.assertEqual(rows[0]["result"],"timeout")
                self.assertEqual(rows[-1]["command"],"get_status_after_stop")
                self.assertEqual(rows[-1]["result"],"responded" if state_code == 10 else "error")
                self.assertIn(state,rows[-1]["detail"])

    def test_stopped_status_allows_one_refresh_after_manual_mode_report(self):
        with TemporaryDirectory() as folder:
            client = NativeMiio("192.0.2.1","0"*32,sequence_file=Path(folder)/"seq.json")
            manual = Mock(state_code=7,state="Manual mode",error_code=0)
            paused = Mock(state_code=10,state="Paused",error_code=0)
            with patch.object(client.device,"status",side_effect=[manual,paused]) as query:
                self.assertEqual(client._require_stopped_status(),"Paused")
            self.assertEqual(query.call_count,2)


if __name__ == "__main__":
    unittest.main()
