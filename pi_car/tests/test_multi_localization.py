from dataclasses import asdict
import math
import unittest
from unittest.mock import Mock
import numpy as np
from pi_car.common.geometry import Intrinsics
from pi_car.pc.calibration import Calibration
from pi_car.pc.camera import RGBDFrame


class MultiLocalizationTests(unittest.TestCase):
    def setUp(self):
        import cv2
        self.cv2 = cv2
        self.intr = Intrinsics(640,480,600.,600.,320.,240.)
        matrix = np.array([[1,0,0,1],[0,-1,0,.8],[0,0,-1,2],[0,0,0,1.]])
        self.calibration = Calibration(matrix.tolist(),'floor','camera',asdict(self.intr),0.)
        self.configs = [dict(robot_id='A',tag_id=1,tag_size_m=.2,
                             center_in_tag_m=(.03,0.,-.1),heading_offset_rad=.2),
                        dict(robot_id='B',tag_id=2,tag_size_m=.16,
                             center_in_tag_m=(0.,0.,0.),heading_offset_rad=-.3)]

    def render(self, included=('A','B')):
        cv2=self.cv2
        image=np.full((480,640),255,np.uint8)
        expected={}
        for config, tx, yaw in zip(self.configs,(-.3,.3),(.25,-.4)):
            r=config['robot_id']
            if r not in included:continue
            tilt,_=cv2.Rodrigues(np.array([.1,0.,0.]))
            z=np.array([[math.cos(yaw),-math.sin(yaw),0],
                        [math.sin(yaw),math.cos(yaw),0],[0,0,1.]])
            rotation=tilt@np.diag([1.,-1.,-1.])@z
            t=np.array([tx,.02,1.5]);s=config['tag_size_m']/2
            obj=np.array([[-s,s,0],[s,s,0],[s,-s,0],[-s,-s,0]])
            rvec,_=cv2.Rodrigues(rotation)
            pixels,_=cv2.projectPoints(obj,rvec,t,self.intr.matrix,np.zeros(5))
            marker=cv2.aruco.generateImageMarker(
                cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11),config['tag_id'],200)
            homography=cv2.getPerspectiveTransform(np.float32([[0,0],[199,0],[199,199],[0,199]]),
                                                    pixels.reshape(4,2).astype(np.float32))
            image=np.minimum(image,cv2.warpPerspective(marker,homography,(640,480),borderValue=255))
            expected[r]=(self.calibration.transform(rotation@np.array(config['center_in_tag_m'])+t),
                         yaw+config['heading_offset_rad'])
        return RGBDFrame(cv2.cvtColor(image,cv2.COLOR_GRAY2RGB),np.ones((480,640)),
                         self.intr,100.,'camera'),expected

    def test_one_detection_multiple_sizes_center_and_heading_offsets(self):
        from pi_car.pc.localization import MultiRobotLocalizer
        localizer=MultiRobotLocalizer(self.calibration,self.configs)
        localizer.first.detector=Mock(wraps=localizer.first.detector)
        frame,expected=self.render()
        poses=localizer.estimate(frame)
        self.assertEqual(localizer.first.detector.detectMarkers.call_count,1)
        for r,(xyz,theta) in expected.items():
            self.assertIsNotNone(poses[r])
            np.testing.assert_allclose([poses[r].x,poses[r].y],xyz[:2],atol=.02)
            self.assertAlmostEqual(poses[r].theta,theta,delta=.06)
            self.assertEqual(poses[r].robot_id,r)

    def test_missing_one_tag_does_not_invalidate_other(self):
        from pi_car.pc.localization import MultiRobotLocalizer
        localizer=MultiRobotLocalizer(self.calibration,self.configs)
        frame,_=self.render(('B',))
        poses=localizer.estimate(frame)
        self.assertIsNone(poses['A']);self.assertIsNotNone(poses['B'])

    def test_duplicate_tag_or_robot_and_missing_configuration_rejected(self):
        from pi_car.pc.localization import MultiRobotLocalizer
        for configs in ([self.configs[0],self.configs[0]],
                        [self.configs[0],dict(self.configs[1],tag_id=1)],
                        [dict(tag_id=1,tag_size_m=.2)]):
            with self.assertRaises(ValueError):MultiRobotLocalizer(self.calibration,configs)


if __name__=='__main__':unittest.main()
