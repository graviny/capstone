"""Regression checks with no windows, real camera, or robot connection."""
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import numpy as np

from navigation_app import App
from navigation_io import CameraWorker


class CameraContinuityTests(unittest.TestCase):
    def test_one_frame_timeout_does_not_end_capture(self):
        worker = CameraWorker.__new__(CameraWorker)
        worker.width,worker.height = 640,480
        worker.lock = threading.Lock()
        worker.closing = threading.Event()
        worker.packet,worker.error = None,"connecting"
        capture = Mock(frame_number=4,depth_scale=.001)
        capture._depth_image = np.zeros((4,4),np.uint16)
        def valid_frame():
            worker.closing.set()
            return True,np.zeros((4,4,3),np.uint8)
        results = iter([RuntimeError("Frame didn't arrive within 5000"),valid_frame])
        def read():
            result = next(results)
            if isinstance(result,Exception):
                raise result
            return result()
        capture.read.side_effect = read
        with patch("realsense_camera.RealSenseCapture",return_value=capture):
            worker._run()
        packet,error = worker.latest()
        self.assertEqual(packet["number"],4)
        self.assertEqual(error,"")
        self.assertEqual(capture.read.call_count,2)

    def test_duplicate_frame_is_skipped_and_next_new_frame_is_published(self):
        worker = CameraWorker.__new__(CameraWorker)
        worker.width,worker.height = 640,480
        worker.lock = threading.Lock()
        worker.closing = threading.Event()
        worker.packet,worker.error = None,"connecting"
        capture = Mock()
        capture.depth_scale = .001
        capture._depth_image = np.zeros((4,4),np.uint16)
        numbers = iter([10,10,11])
        def read():
            capture.frame_number = next(numbers)
            if capture.frame_number==11:
                worker.closing.set()
            return True,np.zeros((4,4,3),np.uint8)
        capture.read.side_effect = read
        with patch("realsense_camera.RealSenseCapture",return_value=capture),patch(
                "navigation_io.time.monotonic",side_effect=[100.,101.]) as clock:
            worker._run()
        packet,error = worker.latest()
        self.assertEqual(error,"")
        self.assertEqual(packet["number"],11)
        self.assertEqual(packet["stamp"],101.)
        # The repeated image did not acquire a fresh observation timestamp.
        self.assertEqual(clock.call_count,2)
        capture.release.assert_called_once()


class FrozenCalibrationTests(unittest.TestCase):
    def make_app(self):
        app = App.__new__(App)
        app.closed = False
        app.packet = {"frame":np.zeros((480,640,3),np.uint8)}
        app.frozen = dict(app.packet)
        app.points = []
        app.video_transform = (20,30,.5)
        app.pending_calibration = None
        app.notice,app.status,app.save_button = Mock(),Mock(),Mock()
        app.start_button,app.root = Mock(),Mock()
        app._show_video,app.stop = Mock(),Mock()
        return app

    def test_click_immediately_marks_frozen_image_without_next_camera_frame(self):
        app = self.make_app()
        app.video_click(SimpleNamespace(x=70,y=80))
        self.assertEqual(app.points,[(100.,100.)])
        app._show_video.assert_called_once_with(app.frozen["frame"])
        self.assertIn("1/4",app.notice.set.call_args.args[0])

    def test_frozen_image_still_repaints_if_camera_disconnects(self):
        app = self.make_app()
        app.camera = Mock()
        app.camera.latest.return_value = (None,"camera disconnected")
        app.tick()
        app.stop.assert_called_once()
        app._show_video.assert_called_once_with(app.frozen["frame"])
        app.start_button.configure.assert_called_once_with(state="disabled")

    def test_fourth_point_remains_visible_even_when_geometry_is_invalid(self):
        app = self.make_app()
        app.points = [(30.,30.),(500.,30.),(500.,400.)]
        app.fields = {key:Mock(get=Mock(return_value=value)) for key,value in
                      [("width","1.2"),("height","1.0"),("marker_height",".12")]}
        app.frozen.update(ray=Mock(),metadata={})
        with patch("navigation_app.FloorCalibration.fit",side_effect=ValueError("invalid rectangle")):
            app.video_click(SimpleNamespace(x=35,y=230))
        self.assertEqual(len(app.points),4)
        app._show_video.assert_called_once_with(app.frozen["frame"])
        self.assertIn("invalid rectangle",app.notice.set.call_args.args[0])


if __name__ == "__main__":
    unittest.main()
