import math
import numpy as np
from pi_car.common.messages import Pose


class AprilTagLocalizer:
    def __init__(self, calibration, tag_id, tag_size_m, robot_id="B",
                 center_in_tag_m=(0., 0., 0.), heading_offset_rad=0., max_reprojection_px=2.,
                 tag_family='apriltag_36h11'):
        import cv2
        if not math.isfinite(tag_size_m) or tag_size_m <= 0:
            raise ValueError("Measure the black square edge; size must be positive")
        self.cv2, self.calibration = cv2, calibration
        dictionaries={'apriltag_36h11':cv2.aruco.DICT_APRILTAG_36h11,
                      'aruco_4x4_50':cv2.aruco.DICT_4X4_50}
        if tag_family not in dictionaries:
            raise ValueError(f'Unsupported tag family: {tag_family}')
        self.tag_id, self.tag_size_m, self.robot_id = tag_id, tag_size_m, robot_id
        self.tag_family=tag_family
        self.offset = np.asarray(center_in_tag_m, float)
        self.heading_offset = heading_offset_rad
        self.max_error = max_reprojection_px
        dictionary = cv2.aruco.getPredefinedDictionary(dictionaries[tag_family])
        parameters=cv2.aruco.DetectorParameters()
        if tag_family=='aruco_4x4_50':
            parameters.cornerRefinementMethod=cv2.aruco.CORNER_REFINE_SUBPIX
            # The 12cm robot tag is ~30px from the ceiling camera. With the default
            # 4px/cell bit sampling, a rotated tag was missed at ~44% of headings
            # (POSE_TIMEOUT mid-pivot); 8px/cell decoded all tested headings.
            parameters.perspectiveRemovePixelPerCell=8
        self.detector = cv2.aruco.ArucoDetector(dictionary, parameters)

    def detect(self, gray):
        if self.tag_family!='aruco_4x4_50':
            corners,ids,_=self.detector.detectMarkers(gray)
            return list(corners),[] if ids is None else [int(i) for i in ids.ravel()]
        # 2x upscale for sub-pixel corners; fall back to native resolution for
        # any marker the upscaled pass missed.
        found=[]
        for scale in (2,1):
            image=gray if scale==1 else self.cv2.resize(gray,None,fx=scale,fy=scale,
                                                        interpolation=self.cv2.INTER_CUBIC)
            corners,ids,_=self.detector.detectMarkers(image)
            if ids is not None:
                known={tag for _,tag in found}
                found+=[(corner/scale,int(tag)) for corner,tag in zip(corners,ids.ravel())
                        if int(tag) not in known]
        return [corner for corner,_ in found],[tag for _,tag in found]

    def estimate(self, frame):
        """None means detection/quality failure; never reuse an old valid pose."""
        self.calibration.check_frame(frame)
        cv2 = self.cv2
        corners, ids = self.detect(cv2.cvtColor(frame.rgb, cv2.COLOR_RGB2GRAY))
        if ids.count(self.tag_id) != 1:
            return None
        image_points = corners[ids.index(self.tag_id)].reshape(4, 2)
        return self.estimate_corners(frame, image_points)

    def estimate_corners(self, frame, image_points):
        """Shared pose solver; multi-robot detection calls this without re-detection."""
        self.calibration.check_frame(frame)
        cv2 = self.cv2
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


class MultiRobotLocalizer:
    """One detection per image, one independently validated pose per configured robot."""
    def __init__(self, calibration, robots):
        self.localizers = {}
        tags = set()
        for config in robots:
            config = dict(config)
            if not all(k in config for k in ("robot_id", "tag_id", "tag_size_m")):
                raise ValueError("Each robot requires robot_id, tag_id, tag_size_m")
            if type(config["tag_id"]) is not int or config["tag_id"] < 0:
                raise ValueError("Tag ID must be a nonnegative integer")
            config.pop("clearance_m", None)
            localizer = AprilTagLocalizer(calibration, **config)
            tag_key=(localizer.tag_family,localizer.tag_id)
            if localizer.robot_id in self.localizers or tag_key in tags:
                raise ValueError("robot_id and tag family/ID must be unique")
            if (localizer.offset.shape != (3,) or not np.isfinite(localizer.offset).all()
                    or not math.isfinite(localizer.heading_offset)):
                raise ValueError("Invalid robot center/heading offset")
            self.localizers[localizer.robot_id] = localizer
            tags.add(tag_key)
        if not self.localizers:
            raise ValueError("Configure at least one robot")
        self.first = next(iter(self.localizers.values()))
        self.last_detection_ids=[]
        self.last_detection_keys=set()

    def estimate(self, frame):
        self.first.calibration.check_frame(frame)
        cv2 = self.first.cv2
        gray=cv2.cvtColor(frame.rgb, cv2.COLOR_RGB2GRAY)
        detections={}
        for localizer in self.localizers.values():
            if localizer.tag_family not in detections:
                detections[localizer.tag_family]=localizer.detect(gray)
        self.last_detection_keys={(family,int(tag_id)) for family,(_,ids) in detections.items()
                                  for tag_id in ids}
        self.last_detection_ids=[tag_id for _,tag_id in sorted(self.last_detection_keys)]
        results = {}
        for robot_id, localizer in self.localizers.items():
            corners,ids=detections[localizer.tag_family]
            pose = None
            if ids.count(localizer.tag_id) == 1:
                try:
                    pose = localizer.estimate_corners(
                        frame, corners[ids.index(localizer.tag_id)].reshape(4, 2))
                except (ValueError, cv2.error):
                    pass
            results[robot_id] = pose
        return results
