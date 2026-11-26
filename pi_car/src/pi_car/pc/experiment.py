"""Repeatable captured-data experiment; never sends motion commands."""
from collections import deque
from pathlib import Path
import json
import math
import numpy as np
from pi_car.common.geometry import transform_points
from pi_car.pc.floor import estimate_floor
from pi_car.pc.mapping import build_grid
from pi_car.vehicle.planning import plan


def _farthest(grid, seed):
    distance, queue = {seed: 0}, deque([seed])
    while queue:
        x, y = queue.popleft()
        for neighbor in ((x+1,y), (x-1,y), (x,y+1), (x,y-1)):
            if neighbor not in distance and grid.is_free(neighbor):
                distance[neighbor] = distance[(x,y)] + 1
                queue.append(neighbor)
    return max(distance, key=distance.get), distance


def choose_test_path(navigation):
    """Two distant free cell centers in the largest connected component."""
    unseen = set(map(tuple, np.argwhere(navigation.cells == 0)[:, ::-1].tolist()))
    largest = {}
    while unseen:
        _, component = _farthest(navigation, min(unseen))
        unseen.difference_update(component)
        if len(component) > len(largest):
            largest = component
    if len(largest) < 2:
        raise ValueError("Not enough connected free space for the configured vehicle clearance")
    first, _ = _farthest(navigation, min(largest))
    last, _ = _farthest(navigation, first)
    start, goal = navigation.cell_to_world(*first), navigation.cell_to_world(*last)
    return start, goal, plan(navigation, start, goal)


def run_experiment(frame, roi, output, *, clearance_m=.2, resolution_m=.03, robot_config=None):
    import matplotlib.pyplot as plt
    from matplotlib.colors import ListedColormap, BoundaryNorm
    from matplotlib.patches import Rectangle
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    calibration, diagnostics = estimate_floor(frame, roi)
    calibration.save(output / "calibration.json")
    yy, xx = np.indices(frame.depth_m.shape)
    valid = np.isfinite(frame.depth_m) & (frame.depth_m > 0)
    points = calibration.transform(frame.intrinsics.deproject(
        np.column_stack((xx[valid], yy[valid])), frame.depth_m[valid]))
    heights = np.full(frame.depth_m.shape, np.nan)
    heights[valid] = points[:, 2]
    floor = points[np.abs(points[:, 2]) <= .025]
    lower = np.floor(floor[:, :2].min(0)/resolution_m)*resolution_m - resolution_m
    upper = np.ceil(floor[:, :2].max(0)/resolution_m)*resolution_m + resolution_m
    config = dict(width_m=float(upper[0]-lower[0]), height_m=float(upper[1]-lower[1]),
                  origin_xy=lower.tolist(), resolution_m=resolution_m, min_hits=3,
                  obstacle_max_m=3.)
    raw = build_grid(frame, calibration, **config)
    raw.save(output / "map")
    navigation = raw.inflated(clearance_m)
    navigation.save(output / "navigation_map")
    (output / "mapping_config.json").write_text(json.dumps(config, indent=2))
    start = goal = None
    path = []
    planning_error = None
    try:
        start, goal, path = choose_test_path(navigation)
    except ValueError as error:
        planning_error = str(error)
    pose = None
    if robot_config is not None:
        from pi_car.pc.localization import AprilTagLocalizer
        pose = AprilTagLocalizer(calibration, **robot_config).estimate(frame)
    payload = dict(frame_id=raw.metadata.frame_id, map_version=raw.metadata.version,
                   type="goal", schema_version=1, goal=goal,
                   start=start, source="automatically selected TEST points, not vehicle pose")
    (output / "goal.json").write_text(json.dumps(payload, indent=2))
    (output / "path.json").write_text(json.dumps(dict(**payload, path=path), indent=2))
    (output / "pose.json").write_text(json.dumps(pose.to_dict() if pose else dict(valid=False), indent=2))
    report = dict(**diagnostics, depth_valid_fraction=float(valid.mean()),
                  map_shape=list(raw.cells.shape), resolution_m=resolution_m,
                  capture_frames=frame.frame_count,
                  free_cells=int(np.sum(raw.cells == 0)),
                  occupied_cells=int(np.sum(raw.cells == 1)),
                  unknown_cells=int(np.sum(raw.cells == -1)),
                  navigable_cells=int(np.sum(navigation.cells == 0)), clearance_m=clearance_m,
                  start_xy=start, goal_xy=goal, waypoint_count=len(path),
                  path_length_m=sum(math.dist(a,b) for a,b in zip(path,path[1:])),
                  planning_error=planning_error, apriltag_detected=pose is not None,
                  limitations=["No surveyed world references or independent accuracy measurement",
                               "Occlusions and depth holes remain unknown",
                               "Test path endpoints are not a measured robot pose",
                               "No vehicle motion was performed"])
    (output / "report.json").write_text(json.dumps(report, indent=2))

    # Show floor selection, metric height and identical test points in RGB/map.
    fig, axes = plt.subplots(1, 3, figsize=(18, 6))
    axes[0].imshow(frame.rgb)
    tint = np.zeros((*heights.shape, 4))
    tint[np.isfinite(heights) & (np.abs(heights) <= .025)] = [0., 1., .4, .3]
    axes[0].imshow(tint)
    axes[0].add_patch(Rectangle((roi[0],roi[1]),roi[2]-roi[0],roi[3]-roi[1],
                                fill=False,edgecolor="yellow",lw=2))
    axes[0].set_title("Actual RGB | green: estimated floor | yellow: fitting ROI")
    image = axes[1].imshow(heights, cmap="coolwarm", vmin=-.05, vmax=.4)
    axes[1].set_title(f"Height above floor | plane fit RMS {diagnostics['plane_rmse_m']*1000:.1f} mm")
    fig.colorbar(image, ax=axes[1], label="Height (m)", fraction=.046)
    m = raw.metadata
    extent = [m.origin_xy[0], m.origin_xy[0]+m.width*m.resolution_m,
              m.origin_xy[1], m.origin_xy[1]+m.height*m.resolution_m]
    axes[2].imshow(raw.cells,origin="lower",extent=extent,
                    cmap=ListedColormap(["#9499a1","#fafafa","#20252c"]),
                    norm=BoundaryNorm([-1.5,-.5,.5,1.5],3))
    inflated = np.ma.masked_where((raw.cells != 0) | (navigation.cells == 0),
                                  np.ones(raw.cells.shape))
    axes[2].imshow(inflated,origin="lower",extent=extent,
                    cmap=ListedColormap(["#efb366"]),vmin=0,vmax=1,alpha=.7)
    if path:
        axes[2].plot(*zip(*path),color="#3486d8",lw=2)
        inverse = np.linalg.inv(np.asarray(calibration.camera_to_world))
        for xy, label, color in ((start,"S (test)","blue"),(goal,"G (test)","red")):
            axes[2].plot(*xy,"o",color=color)
            axes[2].annotate(label,xy,xytext=(5,5),textcoords="offset points",color=color)
            camera = transform_points(inverse, [*xy,0.])
            uv = (camera[:2]/camera[2]) * [frame.intrinsics.fx,frame.intrinsics.fy]
            uv += [frame.intrinsics.cx,frame.intrinsics.cy]
            axes[0].plot(*uv,"o",color=color)
            axes[0].annotate(label,uv,color=color)
    axes[2].set(xlabel="Local X (m)",ylabel="Local Y (m)",
                title=f"A* preview | clearance {clearance_m:.2f}m | {len(path)} points")
    fig.suptitle("REAL CAPTURE EXPERIMENT — local depth frame, no robot motion / no accuracy ground truth")
    fig.tight_layout()
    fig.savefig(output / "experiment.png",dpi=150)
    plt.close(fig)
    return report
