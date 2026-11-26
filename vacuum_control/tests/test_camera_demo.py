"""Deterministic marker checks with synthetic images, without any camera."""

from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

import cv2
import numpy as np

from camera_demo import MarkerTracker, make_marker, pixel_pose


class MarkerTests(unittest.TestCase):
    def test_generated_print_marker_is_detectable(self):
        with TemporaryDirectory() as temporary:
            png, svg = make_marker(temporary)
            frame = cv2.imread(str(png))
            result, _, _ = MarkerTracker().detect(frame, 1)
            self.assertTrue(result["valid"])
            self.assertAlmostEqual(result["heading_image_deg"], 90, delta=1)
            self.assertIn('width="180mm"', svg.read_text(encoding="utf-8"))

    def test_lost_marker_clears_coordinates(self):
        tracker = MarkerTracker()
        with TemporaryDirectory() as temporary:
            png, _ = make_marker(temporary)
            visible, _, _ = tracker.detect(cv2.imread(str(png)), 1)
        lost, _, _ = tracker.detect(np.full((500, 500, 3), 255, np.uint8), 2)
        self.assertTrue(visible["valid"])
        self.assertFalse(lost["valid"])
        self.assertNotIn("u_px", lost)

    def test_duplicate_id_is_not_accepted_as_robot_position(self):
        marker = cv2.aruco.generateImageMarker(
            cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50), 0, 180)
        frame = np.full((300, 600), 255, dtype=np.uint8)
        frame[60:240, 40:220] = marker
        frame[60:240, 360:540] = marker
        result, _, _ = MarkerTracker().detect(frame, 1)
        self.assertFalse(result["valid"])
        self.assertEqual(result["reason"], "duplicate_marker_id")

    def test_rotated_marker_front_direction(self):
        result = pixel_pose([[200, 100], [200, 300], [0, 300], [0, 100]])
        self.assertAlmostEqual(result["u_px"], 100)
        self.assertAlmostEqual(result["v_px"], 200)
        self.assertAlmostEqual(result["heading_image_deg"], 0)


if __name__ == "__main__":
    unittest.main()
