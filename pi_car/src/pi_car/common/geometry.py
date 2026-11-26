from dataclasses import dataclass
import numpy as np


@dataclass
class Intrinsics:
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float

    @property
    def matrix(self):
        return np.array([[self.fx, 0, self.cx], [0, self.fy, self.cy], [0, 0, 1.]])

    def deproject(self, pixels, depth):
        """Rectified pinhole pixels + optical-axis depth (m) → camera XYZ."""
        uv = np.asarray(pixels, dtype=float)
        z = np.asarray(depth, dtype=float)
        return np.stack(((uv[..., 0] - self.cx) * z / self.fx,
                         (uv[..., 1] - self.cy) * z / self.fy, z), axis=-1)


def transform_points(matrix, points):
    matrix = np.asarray(matrix)
    return np.asarray(points) @ matrix[:3, :3].T + matrix[:3, 3]


def fit_rigid(camera_points, world_points):
    """Least-squares metric camera→world transform, including planar floor points."""
    a, b = np.asarray(camera_points, float), np.asarray(world_points, float)
    if a.shape != b.shape or a.ndim != 2 or a.shape[1] != 3 or len(a) < 4:
        raise ValueError("At least four paired XYZ reference points are required")
    if not np.isfinite(a).all() or not np.isfinite(b).all():
        raise ValueError("Reference points must be finite")
    ac, bc = a - a.mean(0), b - b.mean(0)
    if np.linalg.matrix_rank(ac, tol=1e-5) < 2 or np.linalg.matrix_rank(bc, tol=1e-5) < 2:
        raise ValueError("Reference points must not be collinear")
    u, _, vt = np.linalg.svd(ac.T @ bc)
    rotation = vt.T @ np.diag([1., 1., np.linalg.det(vt.T @ u.T)]) @ u.T
    result = np.eye(4)
    result[:3, :3] = rotation
    result[:3, 3] = b.mean(0) - rotation @ a.mean(0)
    rmse = np.sqrt(np.mean(np.sum((transform_points(result, a) - b) ** 2, axis=1)))
    return result, float(rmse)
