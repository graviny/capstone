import math
import numpy as np
from pi_car.common.messages import Pose


class AprilTagLocalizer:
    def __init__(self, calibration, tag_id, tag_size_m, robot_id="B",
                 center_in_tag_m=(0., 0., 0.), heading_offset_rad=0., max_reprojection_px=2.):
        import cv2
        if not math.isfinite(tag_size_m) or tag_size_m <= 0:
            raise ValueError("Measure the black square edge; size must be positive")
        self.cv2, self.calibration = cv2, calibration
        self.tag_id, self.tag_size_m, self.robot_id = tag_id, tag_size_m, robot_id
        self.offset = np.asarray(center_in_tag_m, float)
        self.heading_offset = heading_offset_rad
        self.max_error = max_reprojection_px
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
        self.detector = cv2.aruco.ArucoDetector(dictionary, cv2.aruco.DetectorParameters())

    def estimate(self, frame):
        """None means detection/quality failure; never reuse an old valid pose."""
        self.calibration.check_frame(frame)
        cv2 = self.cv2
        corners, ids, _ = self.detector.detectMarkers(cv2.cvtColor(frame.rgb, cv2.COLOR_RGB2GRAY))
        if ids is None or np.count_nonzero(ids == self.tag_id) != 1:
            return None
        image_points = corners[list(ids.ravel()).index(self.tag_id)].reshape(4, 2)
        s = self.tag_size_m / 2
        # IPPE_SQUARE order: top-left, top-right, bottom-right, bottom-left.
        obj = np.array([[-s, s, 0], [s, s, 0], [s, -s, 0], [-s, -s, 0]], dtype=float)
        _, rvecs, tvecs, _ = cv2.solvePnPGeneric(
            obj, image_points, frame.intrinsics.matrix, np.zeros(5),
            flags=cv2.SOLVEPNP_IPPE_SQUARE)
        candidates = []
        world_camera = np.asarray(self.calibration.camera_to_world)
        for rvec, tvec in zip(rvecs, tvecs):
            rotation, _ = cv2.Rodrigues(rvec)
            translation = tvec.ravel()
            camera_points = obj @ rotation.T + translation
            if not np.isfinite(camera_points).all() or (camera_points[:, 2] <= 0).any():
                continue
            world_rotation = world_camera[:3, :3] @ rotation
            if world_rotation[2, 2] <= .5:  # printed tag face must point upward
                continue
            projected, _ = cv2.projectPoints(obj, rvec, tvec, frame.intrinsics.matrix, np.zeros(5))
            error = float(np.sqrt(np.mean(np.sum((projected.reshape(4, 2)-image_points)**2, axis=1))))
            if error > self.max_error:
                continue
            center = self.calibration.transform(rotation @ self.offset + translation)
            heading = math.atan2(world_rotation[1, 0], world_rotation[0, 0]) + self.heading_offset
            heading = math.atan2(math.sin(heading), math.cos(heading))
            candidates.append((error, Pose(self.robot_id, float(center[0]), float(center[1]),
                                            heading, frame.timestamp, self.calibration.frame_id)))
        return min(candidates, key=lambda item: item[0])[1] if candidates else None
