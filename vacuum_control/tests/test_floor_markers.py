import unittest
from unittest.mock import Mock

import cv2
import numpy as np

from camera_demo import MarkerTracker
from floor_markers import FLOOR_IDS, FLOOR_SIDE_MM, calibrate_floor_markers, floor_corners
from navigation_app import Session
from navigation_core import OccupancyGrid
from navigation_sim import SimulatedCamera


class FloorMarkerTests(unittest.TestCase):
    def test_small_rectangle_can_calibrate_while_navigation_respects_clearance(self):
        camera = SimulatedCamera()
        camera.camera = np.array([.30,.24,.75])
        camera.t = -camera.R@camera.camera
        packet = {"frame":None,"ray":camera.ray,"metadata":camera.metadata}
        points = camera.project(floor_corners(.61,.47,FLOOR_SIDE_MM).reshape(-1,3))
        detector = Mock()
        detector.detectMarkers.return_value = (points.astype(np.float32).reshape(4,1,4,2),
                                                np.asarray(FLOOR_IDS).reshape(-1,1),[])
        calibration,_ = calibrate_floor_markers(detector,packet,.61,.47,.08,FLOOR_SIDE_MM)
        np.testing.assert_allclose(calibration.camera,camera.camera,atol=1e-5)
        grid = OccupancyGrid(calibration.width,calibration.height)
        grid.update_points(np.array([(*grid.world((x,y)),0) for y in range(grid.ny) for x in range(grid.nx)]),None,1)
        self.assertEqual(grid.plan((.25,.24),(.38,.24)),[])

    def test_detects_four_print_patterns_and_tracks_selected_robot(self):
        camera = SimulatedCamera(marker_id=2,show_floor_tags=True)
        packet,_ = camera.latest()
        session = Session(marker_id=2)
        calibration,centres = calibrate_floor_markers(session.tracker.detector,packet,3,2,.12,FLOOR_SIDE_MM)
        self.assertEqual(calibration.data["floor_marker_ids"],list(FLOOR_IDS))
        self.assertEqual(len(centres),4)
        np.testing.assert_allclose(calibration.camera,camera.camera,atol=.015)
        session.calibrate(calibration,packet)
        session.process(packet)
        self.assertTrue(session.healthy)
        self.assertAlmostEqual(session.pose.x,camera.pose.x,delta=.03)

    def test_missing_tags_cannot_be_substituted_by_robot_tag(self):
        camera = SimulatedCamera()
        packet,_ = camera.latest()
        with self.assertRaisesRegex(ValueError,"ID 10, 11, 12, 13"):
            calibrate_floor_markers(MarkerTracker().detector,packet,3,2,.12,FLOOR_SIDE_MM)

    def test_depth_scale_recovers_floor_and_keeps_obstacle(self):
        camera = SimulatedCamera(show_floor_tags=True)
        packet,_ = camera.latest()
        packet["depth"] = np.rint(packet["depth"]*1.116).astype(np.uint16)
        session = Session()
        session.calibrate(camera.calibration,packet)
        session.process(packet)
        self.assertFalse(session.healthy)

        calibration,centres = calibrate_floor_markers(session.tracker.detector,packet,3,2,.12,FLOOR_SIDE_MM)
        self.assertAlmostEqual(calibration.floor_depth_scale,1/1.116,delta=.01)
        session.calibrate(calibration,packet)
        session.process(packet)
        self.assertTrue(session.healthy)
        x,y = session.grid.cell((1.5,1.))
        self.assertTrue(session.grid.obstacles[y,x])
        self.assertTrue(session.grid.plan((session.pose.x,session.pose.y),(2.3,1.45)))

        u,v = map(round,centres[2])
        packet["depth"][v-7:v+8,u-7:u+8] = 1800
        calibration,_ = calibrate_floor_markers(session.tracker.detector,packet,3,2,.12,FLOOR_SIDE_MM)
        self.assertEqual(calibration.floor_depth_scale,1.)

    def test_recovers_tilt_and_rejects_rotated_or_swapped_tag(self):
        camera = SimulatedCamera()
        camera.camera[2] = 3
        camera.R = cv2.Rodrigues(np.array([.06,-.04,.02]))[0]@camera.R
        camera.t = -camera.R@camera.camera
        packet,_ = camera.latest()
        projected = camera.project(floor_corners(3,2,200).reshape(-1,3)).astype(np.float32).reshape(4,1,4,2)
        detector = Mock()
        ids = np.asarray(FLOOR_IDS).reshape(-1,1)
        detector.detectMarkers.return_value = (projected,ids,[])
        calibration,_ = calibrate_floor_markers(detector,packet,3,2,.12,200)
        np.testing.assert_allclose(calibration.camera,camera.camera,atol=1e-4)

        rotated = projected.copy()
        rotated[2] = np.roll(rotated[2],1,axis=1)
        detector.detectMarkers.return_value = (rotated,ids,[])
        with self.assertRaisesRegex(ValueError,"배치 오차.*ID 12"):
            calibrate_floor_markers(detector,packet,3,2,.12,200)

        detector.detectMarkers.return_value = (projected,ids[[0,2,1,3]],[])
        with self.assertRaisesRegex(ValueError,"왼쪽 위 10"):
            calibrate_floor_markers(detector,packet,3,2,.12,200)
