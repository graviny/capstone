"""Depth-only local floor frame for experiments without measured world references."""
from dataclasses import asdict
import uuid
import numpy as np
from pi_car.pc.calibration import Calibration


def pick_floor_roi(frame):
    """Pick a clear floor patch for fitting, not the full driving-area boundary."""
    import matplotlib.pyplot as plt
    fig,ax=plt.subplots()
    ax.imshow(frame.rgb)
    ax.set_title('Click TWO opposite corners of a clear FLOOR patch (no robot/objects); close to cancel')
    try:
        points=plt.ginput(2,timeout=0)
    finally:
        plt.close(fig)
    if len(points)!=2:
        raise ValueError('Floor selection cancelled: two corners are required')
    uv=np.asarray(points,float)
    if uv.shape!=(2,2) or not np.isfinite(uv).all():
        raise ValueError('Invalid floor selection')
    x0,y0=np.floor(uv.min(0)).astype(int)
    x1,y1=np.ceil(uv.max(0)).astype(int)
    if not (0<=x0<x1<=frame.intrinsics.width and 0<=y0<y1<=frame.intrinsics.height):
        raise ValueError('Select a nonempty floor rectangle inside the image')
    return [int(x0),int(y0),int(x1),int(y1)]


def estimate_floor(frame, roi, *, threshold_m=.015, iterations=300, seed=7):
    """RANSAC inside an explicitly selected floor ROI, then SVD refinement.

    This estimates a local metric plane, not an externally surveyed coordinate
    frame. The operator must inspect the resulting floor overlay before use.
    """
    x0, y0, x1, y1 = roi
    if not (0 <= x0 < x1 <= frame.intrinsics.width and
            0 <= y0 < y1 <= frame.intrinsics.height):
        raise ValueError("Floor ROI outside image")
    if threshold_m <= 0 or iterations < 1:
        raise ValueError("Invalid plane fitting settings")
    yy, xx = np.mgrid[y0:y1, x0:x1]
    depth = frame.depth_m[y0:y1, x0:x1]
    valid = np.isfinite(depth) & (depth > 0)
    points = frame.intrinsics.deproject(np.column_stack((xx[valid], yy[valid])), depth[valid])
    if len(points) < 100:
        raise ValueError("Insufficient valid floor depth")
    rng = np.random.default_rng(seed)
    sample = points[rng.choice(len(points), min(6000, len(points)), replace=False)]
    best, normal, offset = 0, None, None
    for _ in range(iterations):
        a, b, c = sample[rng.choice(len(sample), 3, replace=False)]
        n = np.cross(b-a, c-a)
        length = np.linalg.norm(n)
        if length < 1e-8:
            continue
        n /= length
        d = -n @ a
        count = np.count_nonzero(np.abs(sample @ n + d) < threshold_m)
        if count > best:
            best, normal, offset = count, n, d
    if best < len(sample)*.5:
        raise ValueError("No dominant plane: select a clearer floor ROI")
    mask = np.abs(points @ normal + offset) < threshold_m
    for _ in range(3):
        center = points[mask].mean(0)
        _, _, vt = np.linalg.svd(points[mask] - center, full_matrices=False)
        normal = vt[-1]
        if normal @ center > 0:
            normal = -normal  # floor +Z toward camera, camera lies above plane
        offset = -normal @ center
        mask = np.abs(points @ normal + offset) < threshold_m
    if mask.mean() < .5:
        raise ValueError("Refined plane lost support")
    rmse = float(np.sqrt(np.mean((points[mask] @ normal + offset)**2)))
    origin = np.median(points[mask], axis=0)
    origin -= (normal @ origin + offset)*normal
    axis_x = np.array([1., 0., 0.]) - normal[0]*normal
    if np.linalg.norm(axis_x) < .1:
        raise ValueError("Camera right axis is nearly perpendicular to floor")
    axis_x /= np.linalg.norm(axis_x)
    axis_y = np.cross(normal, axis_x)
    matrix = np.eye(4)
    matrix[:3, :3] = np.stack((axis_x, axis_y, normal))
    matrix[:3, 3] = -matrix[:3, :3] @ origin
    calibration = Calibration(matrix.tolist(), str(uuid.uuid4()), frame.serial,
                              asdict(frame.intrinsics), rmse)
    diagnostics = dict(method="depth_roi_ransac_local_frame", floor_roi=list(roi),
                       inlier_fraction=float(mask.mean()), plane_rmse_m=rmse,
                       camera_height_m=float(offset), threshold_m=threshold_m,
                       externally_measured=False,
                       origin="projection of median floor inlier in selected ROI",
                       axes="X=camera right projected onto floor; Y=Z cross X; Z=toward camera")
    return calibration, diagnostics
