from dataclasses import asdict, dataclass
from pathlib import Path
import json
import uuid
import numpy as np
from pi_car.common.geometry import fit_rigid, transform_points


@dataclass
class Calibration:
    camera_to_world: list
    frame_id: str
    serial: str
    intrinsics: dict
    rmse_m: float

    def check_frame(self, frame):
        current=asdict(frame.intrinsics)
        if (self.serial != frame.serial or any(self.intrinsics[k]!=current[k] for k in ('width','height'))):
            raise ValueError("Camera serial/resolution changed: recalibrate")
        # Extrinsics describe the fixed RGB optical frame, not a specific K.
        # Live deprojection/PnP use the current frame K. The D455 can report small
        # focal updates across starts; exact dictionary equality rejected these.
        # Keep a conservative guard against materially different profiles.
        for key in ('fx','fy'):
            reference=self.intrinsics[key]
            value=current[key]
            if (not np.isfinite(value) or value<=0 or reference<=0
                    or abs(value/reference-1.)>.005):
                raise ValueError(f"Camera {key} changed over 0.5%: recalibrate")
        for key in ('cx','cy'):
            if not np.isfinite(current[key]) or abs(current[key]-self.intrinsics[key])>1.:
                raise ValueError("Camera principal point changed over 1 pixel: recalibrate")

    def transform(self, points):
        return transform_points(self.camera_to_world, points)

    def save(self, path):
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        Path(path).write_text(json.dumps(asdict(self), indent=2), encoding="utf-8")

    @classmethod
    def load(cls, path):
        result = cls(**json.loads(Path(path).read_text()))
        matrix = np.asarray(result.camera_to_world, float)
        if (matrix.shape != (4, 4) or not np.isfinite(matrix).all()
                or not np.allclose(matrix[3], [0, 0, 0, 1])
                or not np.allclose(matrix[:3, :3].T @ matrix[:3, :3], np.eye(3), atol=1e-5)
                or not np.isclose(np.linalg.det(matrix[:3, :3]), 1)):
            raise ValueError("Invalid camera_to_world rigid transform")
        return result


def calibrate(frame, pixels, world_xy, max_rmse_m=.03):
    """Use measured floor references (Z=0); these define the floor plane."""
    if len(pixels) != len(world_xy):
        raise ValueError("Click and reference point counts disagree")
    camera = []
    for u, v in pixels:
        x, y = int(round(u)), int(round(v))
        if not 0 <= x < frame.intrinsics.width or not 0 <= y < frame.intrinsics.height:
            raise ValueError("Reference pixel outside image")
        patch = frame.depth_m[max(0, y-2):y+3, max(0, x-2):x+3]
        valid = patch[np.isfinite(patch) & (patch > 0)]
        if len(valid) < 5:
            raise ValueError("Reference has insufficient depth; choose another floor point")
        camera.append(frame.intrinsics.deproject([u, v], np.median(valid)))
    world = np.column_stack((world_xy, np.zeros(len(world_xy))))
    matrix, rmse = fit_rigid(camera, world)
    if rmse > max_rmse_m:
        raise ValueError(f"Calibration RMSE {rmse:.3f}m exceeds {max_rmse_m:.3f}m")
    # World must be right-handed with camera above the floor, +Z upward.
    if matrix[2, 3] <= 0:
        raise ValueError("Camera is below world floor: check reference order and axis handedness")
    return Calibration(matrix.tolist(), str(uuid.uuid4()), frame.serial,
                       asdict(frame.intrinsics), rmse)
