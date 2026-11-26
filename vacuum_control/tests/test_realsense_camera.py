"""SDK-independent tests; never enumerate or stream actual hardware."""

from types import SimpleNamespace
import unittest
from unittest.mock import Mock

import numpy as np

from realsense_camera import RealSenseCapture, median_depth_m


def fake_sdk():
    rs = Mock()
    device = Mock()
    values = {rs.camera_info.name: "Intel RealSense D455",
              rs.camera_info.serial_number: "test-only",
              rs.camera_info.firmware_version: "test",
              rs.camera_info.usb_type_descriptor: "3.2"}
    device.supports.return_value = True
    device.get_info.side_effect = values.__getitem__
    rs.context.return_value.query_devices.return_value = [device]
    pipeline = rs.pipeline.return_value
    profile = pipeline.start.return_value
    profile.get_device.return_value.first_depth_sensor.return_value.get_depth_scale.return_value = 0.001
    return rs, pipeline, profile


class DepthTests(unittest.TestCase):
    def test_depth_uses_scale_and_ignores_invalid_zero_samples(self):
        depth = np.array([[0, 1000, 1000], [0, 1000, 0], [1000, 0, 1000]], dtype=np.uint16)
        self.assertEqual(median_depth_m(depth, 0.001, 1, 1), 1.0)

    def test_missing_or_out_of_frame_depth_is_not_a_position(self):
        depth = np.zeros((3, 3), np.uint16)
        for u, v in [(1, 1), (-1, 0), (3, 0), (float("nan"), 1)]:
            with self.subTest(u=u, v=v):
                self.assertIsNone(median_depth_m(depth, 0.001, u, v))

    def test_absent_device_does_not_start_pipeline(self):
        rs = Mock()
        rs.context.return_value.query_devices.return_value = []
        with self.assertRaisesRegex(RuntimeError, "D455가 보이지"):
            RealSenseCapture(rs_module=rs)
        rs.pipeline.assert_not_called()

    def test_setup_error_after_start_releases_device(self):
        rs, pipeline, profile = fake_sdk()
        profile.get_device.side_effect = RuntimeError("sensor failed")
        with self.assertRaisesRegex(RuntimeError, "sensor failed"):
            RealSenseCapture(rs_module=rs)
        pipeline.stop.assert_called_once()

    def test_release_stops_pipeline_once(self):
        rs, pipeline, _ = fake_sdk()
        capture = RealSenseCapture(rs_module=rs)
        capture.release()
        capture.release()
        pipeline.stop.assert_called_once()
        self.assertFalse(capture.isOpened())

    def test_read_failure_clears_old_depth(self):
        rs, pipeline, _ = fake_sdk()
        capture = RealSenseCapture(rs_module=rs)
        capture._depth_image = np.full((3, 3), 1000, np.uint16)
        pipeline.wait_for_frames.side_effect = RuntimeError("camera disconnected")
        with self.assertRaisesRegex(RuntimeError, "camera disconnected"):
            capture.read()
        self.assertIsNone(capture.depth_at(1, 1))
        capture.release()


if __name__ == "__main__":
    unittest.main()
