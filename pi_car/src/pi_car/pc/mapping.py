from datetime import datetime, timezone
import math
import uuid
import numpy as np
from pi_car.common.grid import GridMetadata, OccupancyGrid


def build_grid(frame, calibration, *, width_m, height_m, resolution_m=.05,
               origin_xy=(0., 0.), floor_tolerance_m=.025,
               obstacle_min_m=.04, obstacle_max_m=1., min_hits=3,
               floor_surface=None, obstacle_fraction=0., floor_below_m=None):
    """Single static capture → occupancy, using calibrated world Z=0 floor.

    Only measured floor surfaces establish free space; holes and occlusions stay
    unknown. Obstacles win over floor observations within a cell. No hole filling.
    floor_surface: optional quadratic floor-height coefficients (pc.floor) that
    are subtracted before thresholding. obstacle_fraction: an obstacle cell also
    needs this share of the cell's depth samples above obstacle_min_m, so a few
    noisy pixels cannot mark an otherwise flat cell. floor_below_m: accept
    depth this far below the floor model as floor (default floor_tolerance_m);
    a room floor has no pits, but D455 depth steps put patches ~2.6 cm low.
    """
    calibration.check_frame(frame)
    if not (0 < floor_tolerance_m < obstacle_min_m < obstacle_max_m):
        raise ValueError("Need 0 < floor tolerance < obstacle min < obstacle max")
    if (not all(math.isfinite(x) and x > 0 for x in (width_m, height_m, resolution_m))
            or min_hits < 1 or not 0 <= obstacle_fraction < 1):
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
    if floor_surface is not None:
        from pi_car.pc.floor import surface_height
        heights = heights - surface_height(floor_surface, points[inside, 0], points[inside, 1])
    totals = np.bincount(indices, minlength=w*h)
    below = floor_tolerance_m if floor_below_m is None else floor_below_m
    if not math.isfinite(below) or below < floor_tolerance_m:
        raise ValueError("floor_below_m must be at least floor_tolerance_m")
    floors = np.bincount(indices[(heights >= -below) & (heights <= floor_tolerance_m)], minlength=w*h)
    obstacles = np.bincount(indices[(heights >= obstacle_min_m) & (heights <= obstacle_max_m)],
                            minlength=w*h)
    cells = np.full(w*h, -1, dtype=np.int8)
    cells[floors >= min_hits] = 0
    cells[(obstacles >= min_hits) & (obstacles >= obstacle_fraction*totals)] = 1
    metadata = GridMetadata(w, h, resolution_m, origin_xy, calibration.frame_id,
                            str(uuid.uuid4()), datetime.now(timezone.utc).isoformat())
    return OccupancyGrid(cells.reshape(h, w), metadata)
