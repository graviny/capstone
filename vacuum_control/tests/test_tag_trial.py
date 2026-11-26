import unittest

import cv2
import numpy as np

from camera_demo import dictionary
from forward_avoid import fit_forward_floor, forward_target
from navigation_app import Session
from navigation_core import Navigator
from navigation_sim import SimulatedCamera
from tag_trial import fit_tag_trial


def trial_fixture(obstacle=False, floor_tags=True):
    camera = SimulatedCamera()
    rgb = np.full((480,640,3),245,np.uint8)
    depth = np.full((480,640),2500,np.uint16)
    tags = [(0,.7,.45,.12,.18)]
    if floor_tags:
        tags.extend([(10,.65,1.95,0,.16),(11,2.5,1.95,0,.16)])
    for marker_id,x,y,height,size in tags:
        w = size/2
        p = camera.project([[x-w,y+w,height],[x+w,y-w,height]])
        (left,top),(right,bottom) = np.rint(p).astype(int)
        block = cv2.aruco.generateImageMarker(dictionary(),marker_id,120)
        rgb[top:bottom,left:right] = cv2.resize(block,(right-left,bottom-top),interpolation=cv2.INTER_NEAREST)[:,:,None]
        if height:
            depth[top:bottom,left:right] = round((2.5-height)*1000)
    if obstacle:
        bounds = camera.project([[.51,1.27,.2],[.91,.99,.2]])
        (left,top),(right,bottom) = np.rint(bounds).astype(int)
        depth[top:bottom,left:right] = 2300
    return {"frame":rgb,"depth":depth,"scale":.001,"stamp":0.,"metadata":camera.metadata,"ray":camera.ray}


class TrialTests(unittest.TestCase):
    def test_forward_without_floor_tags_continues_by_detouring_when_box_appears(self):
        session = Session()
        packet = trial_fixture(floor_tags=False)
        cal = fit_forward_floor(packet,session.tracker.detector,0)
        self.assertAlmostEqual(cal.camera[2],2.5,places=3)
        session.calibrate(cal,packet)
        session.process(packet)
        pose = session.pose
        self.assertTrue(session.healthy)
        goal = forward_target(pose,session.grid,1.3)
        self.assertAlmostEqual(goal[0],pose.x,delta=.03)
        self.assertAlmostEqual(goal[1]-pose.y,1.3,delta=.03)

        nav = Navigator(max_speed=.05,measure=True)
        nav.set_goal(goal)
        nav.step(pose,session.grid,0)
        self.assertIsNone(nav.last_replan)
        nav.running = True
        pose.stamp = .5
        self.assertEqual(len(nav.path),2)
        velocity,rotation = nav.step(pose,session.grid,.5)
        self.assertGreater(velocity,0)
        self.assertEqual(rotation,0)

        blocked = trial_fixture(obstacle=True,floor_tags=False)
        blocked["stamp"] = .6
        session.process(blocked)
        self.assertEqual(nav.step(session.pose,session.grid,.6),(0,0))
        self.assertEqual(nav.path,[])
        self.assertIsNone(nav.last_replan)
        session.pose.stamp = .7
        self.assertEqual(nav.step(session.pose,session.grid,.7),(0,0))
        self.assertGreater(len(nav.path),2)
        self.assertTrue(nav.last_replan.depth_obstacle)
        self.assertEqual(nav.last_replan.route_points,len(nav.path))
        self.assertGreaterEqual(nav.last_replan.finished,nav.last_replan.started)
        session.pose.stamp = 1.1
        _,rotation = nav.step(session.pose,session.grid,1.1)
        self.assertNotEqual(rotation,0)

    def test_real_marker_detection_from_two_floor_tags_and_depth_detour(self):
        session = Session()
        packet = trial_fixture()
        cal,goal,references = fit_tag_trial(packet,session.tracker.detector,0)
        self.assertAlmostEqual(cal.camera[2],2.5,places=3)
        self.assertAlmostEqual(cal.marker_height,.12,places=3)
        session.calibrate(cal,packet)
        session.trial_reference = references
        session.process(packet)
        self.assertTrue(session.healthy)
        start = (session.pose.x,session.pose.y)
        self.assertEqual(session.grid.plan(start,goal),[start,goal])
        navigator = Navigator()
        navigator.set_goal(goal)
        navigator.step(session.pose,session.grid,0)
        navigator.running = True

        with_box = trial_fixture(obstacle=True)
        with_box['stamp'] = .1
        session.process(with_box)
        self.assertTrue(session.healthy)
        self.assertFalse(session.grid.segment_free(start,goal))
        self.assertGreater(len(session.grid.plan(start,goal)),2)
        before = navigator.replans
        self.assertEqual(navigator.step(session.pose,session.grid,.1),(0,0))
        self.assertEqual(navigator.replans,before)
        self.assertEqual(navigator.step(session.pose,session.grid,.2),(0,0))
        self.assertEqual(navigator.replans,before+1)
        self.assertGreater(len(navigator.path),2)

        # ID 11 continues to anchor the map even if the robot covers ID 10.
        with_box['frame'][:,:] = 245
        session.process(with_box)
        self.assertFalse(session.healthy)
        self.assertIsNone(session.pose)
