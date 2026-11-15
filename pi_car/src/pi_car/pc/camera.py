from dataclasses import asdict, dataclass
from pathlib import Path
import json
import time
import numpy as np
from pi_car.common.geometry import Intrinsics


def rectification_maps(rs, intrinsics):
    """Output pinhole pixel → original lens pixel, using the SDK's lens model.

    D455 RGB can report inverse_brown_conrady; passing those coefficients to
    OpenCV's forward Brown model silently produces the wrong geometry.
    Compute once at startup, then use fast cv2.remap for every RGB-D pair.
    """
    intr = intrinsics
    rays = ([float((x-intr.ppx)/intr.fx), float((y-intr.ppy)/intr.fy), 1.]
            for y in range(intr.height) for x in range(intr.width))
    pixels = np.array([rs.rs2_project_point_to_pixel(intr, ray) for ray in rays],
                      dtype=np.float32).reshape(intr.height, intr.width, 2)
    if not np.isfinite(pixels).all():
        raise ValueError("Non-finite lens rectification map")
    return pixels[:, :, 0].copy(), pixels[:, :, 1].copy()


@dataclass
class RGBDFrame:
    rgb: np.ndarray
    depth_m: np.ndarray
    intrinsics: Intrinsics  # RGB and depth both rectified to this optical frame
    timestamp: float
    serial: str

    def save(self, path):
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        np.savez_compressed(path, rgb=self.rgb, depth_m=self.depth_m,
                            metadata=json.dumps(dict(intrinsics=asdict(self.intrinsics),
                                                     timestamp=self.timestamp, serial=self.serial)))

    @classmethod
    def load(cls, path):
        with np.load(path, allow_pickle=False) as data:
            meta = json.loads(str(data["metadata"]))
            return cls(data["rgb"], data["depth_m"], Intrinsics(**meta["intrinsics"]),
                       meta["timestamp"], meta["serial"])


class RealSenseCamera:
    """Align depth to RGB, then rectify BOTH images using the RGB lens model."""

    def __enter__(self):
        import pyrealsense2 as rs
        import cv2
        self.rs, self.cv2 = rs, cv2
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)
        profile = self.pipeline.start(config)
        try:
            for sensor in profile.get_device().query_sensors():
                if sensor.supports(rs.option.global_time_enabled):
                    sensor.set_option(rs.option.global_time_enabled, 1.)
            self.serial = profile.get_device().get_info(rs.camera_info.serial_number)
            self.scale = profile.get_device().first_depth_sensor().get_depth_scale()
            intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
            if intr.model not in (rs.distortion.none, rs.distortion.brown_conrady,
                                  rs.distortion.modified_brown_conrady,
                                  rs.distortion.inverse_brown_conrady):
                raise ValueError(f"Unsupported RGB lens distortion: {intr.model}")
            self.intrinsics = Intrinsics(intr.width, intr.height, intr.fx, intr.fy, intr.ppx, intr.ppy)
            self.maps = rectification_maps(rs, intr)
            self.align = rs.align(rs.stream.color)
            for _ in range(30):
                self.pipeline.wait_for_frames(5000)
        except BaseException:
            self.pipeline.stop()
            raise
        return self

    def read(self):
        frames = self.pipeline.wait_for_frames(1000)
        # Estimate host capture time from SDK frame age; do not stamp after processing.
        color = frames.get_color_frame()
        if not color:
            raise RuntimeError("Missing RGB frame")
        if color.get_frame_timestamp_domain() == self.rs.timestamp_domain.hardware_clock:
            raise RuntimeError("Global camera timestamps unavailable; enable global_time_enabled")
        timestamp = color.get_timestamp() / 1000.
        if abs(time.time() - timestamp) > 1.:
            raise RuntimeError("Camera timestamp not synchronized with host clock")
        frames = self.align.process(frames)
        depth, color = frames.get_depth_frame(), frames.get_color_frame()
        if not depth or not color:
            raise RuntimeError("Missing aligned RGB-D frame")
        rgb = self.cv2.remap(np.asanyarray(color.get_data()), *self.maps,
                             self.cv2.INTER_LINEAR)
        depth_m = self.cv2.remap(np.asanyarray(depth.get_data()).astype(np.float32) * self.scale,
                                 *self.maps, self.cv2.INTER_NEAREST)
        return RGBDFrame(rgb, depth_m, self.intrinsics, timestamp, self.serial)

    def __exit__(self, *_):
        self.pipeline.stop()
