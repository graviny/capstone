from datetime import datetime, timezone
import math
import uuid
import numpy as np
from pi_car.common.grid import GridMetadata, OccupancyGrid


def build_grid(frame, calibration, *, width_m, height_m, resolution_m=.05,
               origin_xy=(0., 0.), floor_tolerance_m=.025,
               obstacle_min_m=.04, obstacle_max_m=1., min_hits=3):
    """Single static capture → occupancy, using calibrated world Z=0 floor.

    Only measured floor surfaces establish free space; holes and occlusions stay
    unknown. Obstacles win over floor observations within a cell. No hole filling.
    """
    calibration.check_frame(frame)
    if not (0 < floor_tolerance_m < obstacle_min_m < obstacle_max_m):
        raise ValueError("Need 0 < floor tolerance < obstacle min < obstacle max")
    if (not all(math.isfinite(x) and x > 0 for x in (width_m, height_m, resolution_m))
            or min_hits < 1):
        raise ValueError("Invalid map size, resolution or min_hits")
    w, h = math.ceil(width_m/resolution_m), math.ceil(height_m/resolution_m)
    yy, xx = np.indices(frame.depth_m.shape)
    depth = frame.depth_m
    valid = np.isfinite(depth) & (depth > 0)
    points = calibration.transform(frame.intrinsics.deproject(
        np.column_stack((xx[valid], yy[valid])), depth[valid]))
    columns = np.floor((points[:, 0] - origin_xy[0])/resolution_m).astype(int)
    rows = np.floor((points[:, 1] - origin_xy[1])/resolution_m).astype(int)
    inside = (columns >= 0) & (columns < w) & (rows >= 0) & (rows < h)
    indices = rows[inside] * w + columns[inside]
    heights = points[inside, 2]
    floors = np.bincount(indices[np.abs(heights) <= floor_tolerance_m], minlength=w*h)
    obstacles = np.bincount(indices[(heights >= obstacle_min_m) & (heights <= obstacle_max_m)],
                            minlength=w*h)
    cells = np.full(w*h, -1, dtype=np.int8)
    cells[floors >= min_hits] = 0
    cells[obstacles >= min_hits] = 1
    metadata = GridMetadata(w, h, resolution_m, origin_xy, calibration.frame_id,
                            str(uuid.uuid4()), datetime.now(timezone.utc).isoformat())
    return OccupancyGrid(cells.reshape(h, w), metadata)
