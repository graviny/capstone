"""Hardware-free geometry, dataflow and fail-stop regression checks."""
from dataclasses import asdict
from pathlib import Path
from types import SimpleNamespace
import json
import math
import tempfile
import time
import unittest
import numpy as np
from pi_car.common.geometry import Intrinsics, fit_rigid, transform_points
from pi_car.common.grid import GridMetadata, OccupancyGrid
from pi_car.common.messages import Mode, MotorCommand, Pose
from pi_car.pc.camera import RGBDFrame, RealSenseCamera, rectification_maps
from pi_car.pc.calibration import Calibration, calibrate
from pi_car.pc.mapping import build_grid
from pi_car.transport.codec import decode_pose, encode_pose
from pi_car.vehicle.hardware import DisabledMotor
from pi_car.vehicle.hardware.adeept import AdeeptUltrasonic, AdeeptMotor
from pi_car.vehicle.perception import RangeReading
from pi_car.vehicle.planning import plan
from pi_car.vehicle.runtime import VehicleRuntime


def grid(cells=None):
    cells = np.zeros((20, 30), dtype=np.int8) if cells is None else cells
    return OccupancyGrid(cells, GridMetadata(cells.shape[1], cells.shape[0], .1,
                                            (0., 0.), "floor-test", "v1", "test"))


def scene():
    intr = Intrinsics(100, 80, 100., 100., 50., 40.)
    # Camera at (1, .8, 2), looking down; right-handed world +Z up.
    transform = np.array([[1, 0, 0, 1], [0, -1, 0, .8], [0, 0, -1, 2], [0, 0, 0, 1.]])
    depth = np.full((80, 100), 2., dtype=np.float32)
    frame = RGBDFrame(np.zeros((80, 100, 3), np.uint8), depth, intr, 100., "test-camera")
    calibration = Calibration(transform.tolist(), "floor-test", "test-camera", asdict(intr), 0.)
    return frame, calibration


class GeometryAndMapTests(unittest.TestCase):
    def test_depth_floor_ransac_rejects_outliers_and_defines_right_handed_frame(self):
        from pi_car.pc.floor import estimate_floor
        frame, _ = scene()
        rng = np.random.default_rng(11)
        frame.depth_m += rng.normal(0, .002, frame.depth_m.shape).astype(np.float32)
        frame.depth_m[10:30, 15:35] = 1.5  # an object inside the floor ROI
        calibration, report = estimate_floor(frame, (10, 5, 90, 75))
        self.assertGreater(report['inlier_fraction'], .8)
        self.assertLess(report['plane_rmse_m'], .005)
        self.assertAlmostEqual(report['camera_height_m'], 2., delta=.01)
        rotation = np.asarray(calibration.camera_to_world)[:3, :3]
        self.assertAlmostEqual(np.linalg.det(rotation), 1.)
        floor_point = frame.intrinsics.deproject([70, 50], 2.)
        self.assertAlmostEqual(calibration.transform(floor_point)[2], 0., delta=.005)

    def test_inverse_lens_rectification_with_sdk(self):
        try:
            import pyrealsense2 as rs
        except ImportError:
            self.skipTest("Optional RealSense SDK not installed")
        intr = rs.intrinsics()
        intr.width, intr.height = 32, 24
        intr.fx, intr.fy, intr.ppx, intr.ppy = 20., 20., 16., 12.
        intr.model = rs.distortion.inverse_brown_conrady
        intr.coeffs = [-.056041, .0657926, -.000277961, .000977528, -.021366]
        map_x, map_y = rectification_maps(rs, intr)
        for x, y in ((0, 0), (31, 23), (16, 12), (5, 18)):
            ray = rs.rs2_deproject_pixel_to_point(intr, [float(map_x[y,x]), float(map_y[y,x])], 1.)
            np.testing.assert_allclose(ray, [(x-16)/20, (y-12)/20, 1.], atol=1e-5)

    def test_camera_read_preserves_global_capture_time_and_rejects_hardware_clock(self):
        # Exercise the actual read path without USB; SDK global times are ms UNIX.
        stamp = time.time() - .05
        domain = ["global"]
        color = SimpleNamespace(get_timestamp=lambda: stamp*1000,
                                get_frame_timestamp_domain=lambda: domain[0],
                                get_data=lambda: np.zeros((2, 3, 3), np.uint8))
        depth = SimpleNamespace(get_data=lambda: np.full((2, 3), 1500, np.uint16))
        frames = SimpleNamespace(get_color_frame=lambda: color, get_depth_frame=lambda: depth)
        camera = RealSenseCamera()
        camera.pipeline = SimpleNamespace(wait_for_frames=lambda timeout: frames)
        camera.rs = SimpleNamespace(timestamp_domain=SimpleNamespace(hardware_clock="hardware"))
        camera.align = SimpleNamespace(process=lambda frame: frame)
        camera.cv2 = SimpleNamespace(INTER_LINEAR=1, INTER_NEAREST=0,
                                     remap=lambda data, *args: data.copy())
        camera.maps, camera.scale, camera.serial = (None, None), .001, "test"
        camera.intrinsics = Intrinsics(3, 2, 2., 2., 1., 1.)
        frame = camera.read()
        # Epoch-sized doubles lose sub-microsecond precision in seconds↔ms.
        self.assertAlmostEqual(frame.timestamp, stamp, delta=1e-6)
        np.testing.assert_allclose(frame.depth_m, 1.5)
        domain[0] = "hardware"
        with self.assertRaises(RuntimeError):
            camera.read()

    def test_floor_calibration_recovers_metric_transform(self):
        frame, expected = scene()
        pixels = [[10, 10], [90, 10], [90, 70], [10, 70]]
        xyz = expected.transform(frame.intrinsics.deproject(pixels, np.full(4, 2.)))
        actual = calibrate(frame, pixels, xyz[:, :2])
        np.testing.assert_allclose(actual.camera_to_world, expected.camera_to_world, atol=1e-7)
        with self.assertRaises(ValueError):
            fit_rigid(np.zeros((4, 3)), np.zeros((4, 3)))

    def test_depth_mapping_obstacle_unknown_and_roundtrip(self):
        frame, calibration = scene()
        frame.depth_m[25:40, 40:55] = 1.8  # box top 20cm above floor
        frame.depth_m[:, :10] = 0  # invalid depth remains unknown
        mapped = build_grid(frame, calibration, width_m=2, height_m=1.6,
                            resolution_m=.05, min_hits=1)
        self.assertTrue((mapped.cells == -1).any())
        self.assertTrue((mapped.cells == 0).any())
        obstacle_xy = calibration.transform(frame.intrinsics.deproject([47, 32], 1.8))[:2]
        x, y = mapped.world_to_cell(*obstacle_xy)
        self.assertEqual(mapped.cells[y, x], 1)
        with tempfile.TemporaryDirectory() as directory:
            mapped.save(directory)
            loaded = OccupancyGrid.load(directory)
            np.testing.assert_array_equal(mapped.cells, loaded.cells)
            frame.save(Path(directory, "frame.npz"))
            np.testing.assert_array_equal(RGBDFrame.load(Path(directory, "frame.npz")).depth_m,
                                          frame.depth_m)

    def test_unknown_blocked_and_path_avoids_wall(self):
        mapped = grid()
        mapped.cells[:15, 15] = -1
        path = plan(mapped, (.25, .25), (2.55, .25))
        self.assertTrue(max(y for _, y in path) >= 1.55)
        self.assertTrue(all(mapped.is_free(mapped.world_to_cell(*point)) for point in path))
        mapped.cells[:, 15] = 1
        with self.assertRaises(ValueError):
            plan(mapped, (.25, .25), (2.55, .25))

    def test_inflation_blocks_boundary_obstacle_and_unknown(self):
        mapped = grid()
        mapped.cells[10, 15] = 1
        mapped.cells[10, 5] = -1
        nav = mapped.inflated(.2)
        self.assertFalse(nav.is_free((1, 1)))
        self.assertFalse(nav.is_free((13, 10)))
        self.assertFalse(nav.is_free((7, 10)))
        self.assertTrue(nav.is_free((22, 10)))
        self.assertEqual(mapped.cells[10, 13], 0)
        self.assertEqual(mapped.world_to_cell(-.01, 0), (-1, 0))

    def test_mismatched_calibration_rejected(self):
        frame, calibration = scene()
        frame.serial = "other"
        with self.assertRaises(ValueError):
            build_grid(frame, calibration, width_m=2, height_m=2)


class RuntimeTests(unittest.TestCase):
    def test_motion_stops_on_pose_loss_range_loss_disconnect_and_obstacle(self):
        motor = DisabledMotor()
        runtime = VehicleRuntime(motor, "B", "floor-test")
        runtime.set_path([(.2, .2), (1., .2)])
        pose = Pose("B", .2, .2, 0., 100., "floor-test")
        reading = RangeReading(1., 0., 5.)
        def tick(p=pose, r=reading, connected=True, now=100.1):
            return runtime.tick(p, r, connected=connected, now_unix=now, now_monotonic=5.1)
        self.assertEqual(tick(), Mode.NORMAL)
        self.assertGreater(motor.last_command.left, 0)
        for kwargs in (dict(p=None), dict(r=None), dict(connected=False), dict(now=101.),
                       dict(r=RangeReading(1., .7, 5.)), dict(r=RangeReading(math.nan, 0., 5.)),
                       dict(p=Pose("B", .2, .2, 0., 100., "different-frame"))):
            self.assertEqual(tick(**kwargs), Mode.EMERGENCY)
            self.assertEqual(motor.last_command, MotorCommand())
        self.assertEqual(tick(r=RangeReading(.1, 0., 5.)), Mode.OBSTACLE)
        self.assertEqual(motor.last_command, MotorCommand())

    def test_wire_pose_roundtrip_and_vendor_units(self):
        pose = Pose("B", 1., 2., math.pi/2, 100., "floor-test")
        self.assertEqual(decode_pose(encode_pose(pose)), pose)
        sensor = AdeeptUltrasonic(SimpleNamespace(checkdist=lambda: 35.))
        self.assertAlmostEqual(sensor.read().distance_m, .35)
        saturated = AdeeptUltrasonic(SimpleNamespace(checkdist=lambda: 200.))
        self.assertTrue(math.isnan(saturated.read().distance_m))

    def test_vendor_motors_disabled_by_default(self):
        calls = []
        vendor = SimpleNamespace(motorStop=lambda: calls.append("stop"))
        adapter = AdeeptMotor(vendor)
        with self.assertRaises(RuntimeError):
            adapter.apply(MotorCommand(.2, .2))
        self.assertEqual(calls, ["stop"])

    def test_vendor_motor_output_preserves_polarity_without_resetting_pwm(self):
        def reset_frequency(*_):
            self.fail("Motor() resets the shared PWM frequency")

        vendor = SimpleNamespace(Motor=reset_frequency)
        for channel, polarity in enumerate((1, -1, 1, -1), start=1):
            setattr(vendor, f"motor{channel}", SimpleNamespace(throttle=0.))
            setattr(vendor, f"M{channel}_Direction", polarity)

        motor = AdeeptMotor(vendor, enabled=True, max_output=.30)
        motor.apply(MotorCommand(.6, -.25))

        self.assertEqual([vendor.motor1.throttle, vendor.motor2.throttle,
                          vendor.motor3.throttle, vendor.motor4.throttle],
                         [.30, -.30, -.25, .25])


class InspectionTests(unittest.TestCase):
    def test_experiment_writes_metric_map_test_goal_and_report(self):
        import matplotlib
        matplotlib.use("Agg")
        from pi_car.pc.experiment import run_experiment
        frame, _ = scene()
        with tempfile.TemporaryDirectory() as directory:
            report = run_experiment(frame, (15,10,85,70), directory, clearance_m=.1,
                                    resolution_m=.05)
            self.assertGreater(report['waypoint_count'], 1)
            self.assertFalse(report['externally_measured'])
            self.assertIsNone(report['planning_error'])
            self.assertTrue(Path(directory, 'experiment.png').exists())
            loaded = OccupancyGrid.load(Path(directory, 'map'))
            goal = json.loads(Path(directory, 'goal.json').read_text())
            self.assertEqual(goal['frame_id'], loaded.metadata.frame_id)

    def test_goal_selection_path_and_invalid_space(self):
        import matplotlib
        matplotlib.use("Agg")
        from pi_car.pc.viewer import MapInspector
        with tempfile.TemporaryDirectory() as directory:
            view = MapInspector(grid(), .1, directory)
            def click(x, y, button):
                view._click(SimpleNamespace(xdata=x, ydata=y, button=button, inaxes=view.map_ax))
            click(.55, .55, 3)
            click(2.45, 1.45, 1)
            self.assertTrue(view.path)
            goal = json.loads(Path(directory, "goal.json").read_text())
            self.assertEqual(goal["frame_id"], "floor-test")
            previous = view.goal
            click(-1., -1., 1)
            self.assertEqual(view.goal, previous)
            view.fig.savefig(Path(directory, "inspection.png"))
            view.plt.close(view.fig)

    def test_apriltag_pose_from_rendered_marker(self):
        import cv2
        from pi_car.pc.localization import AprilTagLocalizer
        frame, calibration = scene()
        # Larger pinhole image with off-axis tilted tag avoids frontoparallel ambiguity.
        frame.intrinsics = Intrinsics(640, 480, 600., 600., 320., 240.)
        calibration.intrinsics = asdict(frame.intrinsics)
        frame.rgb = np.full((480, 640, 3), 255, np.uint8)
        frame.depth_m = np.full((480, 640), 2., np.float32)
        side = .2
        obj = np.array([[-.1, .1, 0], [.1, .1, 0], [.1, -.1, 0], [-.1, -.1, 0]])
        rotation = np.diag([1., -1., -1.])
        # yaw 0.3 in world, slight tilt around camera X
        yaw = np.array([[math.cos(.3), -math.sin(.3), 0],
                        [math.sin(.3), math.cos(.3), 0], [0, 0, 1.]])
        tilt, _ = cv2.Rodrigues(np.array([.1, 0., 0.]))
        rotation = tilt @ rotation @ yaw
        rvec, _ = cv2.Rodrigues(rotation)
        tvec = np.array([.1, .05, 1.5])
        corners, _ = cv2.projectPoints(obj, rvec, tvec, frame.intrinsics.matrix, np.zeros(5))
        marker = cv2.aruco.generateImageMarker(
            cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11), 0, 200)
        homography = cv2.getPerspectiveTransform(np.float32([[0,0],[199,0],[199,199],[0,199]]),
                                                corners.reshape(4, 2).astype(np.float32))
        gray = cv2.warpPerspective(marker, homography, (640,480), borderValue=255)
        frame.rgb = cv2.cvtColor(gray, cv2.COLOR_GRAY2RGB)
        estimator = AprilTagLocalizer(calibration, 0, side)
        pose = estimator.estimate(frame)
        self.assertIsNotNone(pose)
        expected = calibration.transform(tvec)
        self.assertAlmostEqual(pose.x, expected[0], delta=.02)
        self.assertAlmostEqual(pose.y, expected[1], delta=.02)
        self.assertAlmostEqual(pose.theta, .3, delta=.05)
        offset_estimator = AprilTagLocalizer(calibration, 0, side, center_in_tag_m=(.1, 0., 0.))
        shifted = offset_estimator.estimate(frame)
        expected_offset = calibration.transform(rotation @ np.array([.1, 0., 0.]) + tvec)
        self.assertAlmostEqual(shifted.x, expected_offset[0], delta=.02)
        self.assertAlmostEqual(shifted.y, expected_offset[1], delta=.02)
        from pi_car.pc.tagcheck import check_tags
        frame.depth_m[:] = 1.5
        with tempfile.TemporaryDirectory() as directory:
            diagnostics = check_tags(frame, calibration, grid(), directory)
            self.assertEqual(diagnostics['valid_tag_count'], 1)
            observed = diagnostics['tags'][0]
            self.assertEqual(observed['tag_id'], 0)
            self.assertIsNone(observed['measured_tag_size_m'])
            np.testing.assert_allclose(observed['center_xyz_m'], expected, atol=.02)
            frame.rgb[:] = 255
            self.assertEqual(check_tags(frame, calibration, grid(), directory)['valid_tag_count'], 0)
        frame.rgb[:] = 255
        self.assertIsNone(estimator.estimate(frame))


if __name__ == "__main__":
    unittest.main()
