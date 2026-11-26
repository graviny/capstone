"""Captured AprilTag center checks without assuming a printed tag size."""
from pathlib import Path
import json
import math
import numpy as np


def check_tags(frame, calibration, grid, output, tag_size_m=None):
    """Depth gives center position; heading assumes a tag parallel to the floor.

    This is a tag diagnostic, not a vehicle pose (no vehicle center offset).
    PnP is evaluated separately only when a measured edge length is supplied.
    """
    import cv2
    import matplotlib.pyplot as plt
    from matplotlib.colors import ListedColormap, BoundaryNorm
    from pi_car.pc.localization import AprilTagLocalizer
    calibration.check_frame(frame)
    if grid.metadata.frame_id != calibration.frame_id:
        raise ValueError("Tag and map coordinate frames differ")
    detector = cv2.aruco.ArucoDetector(
        cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11),
        cv2.aruco.DetectorParameters())
    corners, ids, _ = detector.detectMarkers(cv2.cvtColor(frame.rgb, cv2.COLOR_RGB2GRAY))
    results = []
    matrix = np.asarray(calibration.camera_to_world)
    for tag_corners, tag_id in zip(corners, [] if ids is None else ids.ravel()):
        uv = tag_corners.reshape(4, 2)
        center_uv = uv.mean(0)
        mask = np.zeros(frame.depth_m.shape, np.uint8)
        cv2.fillConvexPoly(mask, np.rint(uv).astype(np.int32), 1)
        mask = cv2.erode(mask, np.ones((3,3),np.uint8))
        selected = frame.depth_m[(mask != 0) & np.isfinite(frame.depth_m) & (frame.depth_m > 0)]
        result = dict(tag_family="36h11", tag_id=int(tag_id), corners_px=uv.tolist(),
                       center_px=center_uv.tolist(),
                       mean_edge_px=float(np.mean(np.linalg.norm(uv-np.roll(uv,-1,axis=0),axis=1))),
                       depth_sample_count=len(selected), frame_id=calibration.frame_id,
                       timestamp=frame.timestamp, depth_valid=len(selected) >= 5,
                       measured_tag_size_m=tag_size_m,
                       independently_measured_accuracy=False)
        if len(selected) >= 5:
            depth = float(np.median(selected))
            center = calibration.transform(frame.intrinsics.deproject(center_uv, depth))
            # Intersect corner rays with the plane at the observed center height.
            # Appropriate only for the current flat phone/tag experiment.
            rays_camera = frame.intrinsics.deproject(uv, np.ones(4))
            rays_world = rays_camera @ matrix[:3,:3].T
            camera_world = matrix[:3,3]
            factors = (center[2]-camera_world[2])/rays_world[:,2]
            projected = camera_world + rays_world*factors[:,None]
            direction = projected[1,:2] - projected[0,:2]
            heading = math.atan2(direction[1],direction[0])
            result.update(center_xyz_m=center.tolist(), depth_m=depth,
                          depth_iqr_m=float(np.percentile(selected,75)-np.percentile(selected,25)),
                          theta_floor_parallel_rad=heading,
                          theta_floor_parallel_deg=math.degrees(heading),
                          approximate_edge_from_depth_m=float(np.mean(np.linalg.norm(
                              projected-np.roll(projected,-1,axis=0),axis=1))))
        if tag_size_m is not None:
            pose = AprilTagLocalizer(calibration, int(tag_id), tag_size_m,
                                    center_in_tag_m=(0.,0.,0.)).estimate(frame)
            result['pnp_tag_pose'] = pose.to_dict() if pose else None
        results.append(result)
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    report = dict(tags=results, valid_tag_count=sum(r['depth_valid'] for r in results),
                   frame_id=calibration.frame_id,
                   coordinate_frame="new depth-only local floor frame, not previous capture frame",
                   limitations=["Single capture: no temporal stability assessment",
                                "No independently measured position/heading ground truth",
                                "Heading assumes the tag lies parallel to the floor",
                                "Depth and lens calibration errors remain"])
    (output/'tag_report.json').write_text(json.dumps(report,indent=2))
    fig, axes = plt.subplots(1,3,figsize=(17,6))
    axes[0].imshow(frame.rgb)
    axes[0].set_title('Actual captured RGB / detected AprilTag')
    axes[1].imshow(frame.rgb)
    axes[1].set_title('Tag detail (original pixels, enlarged)')
    m = grid.metadata
    extent=[m.origin_xy[0],m.origin_xy[0]+m.width*m.resolution_m,
            m.origin_xy[1],m.origin_xy[1]+m.height*m.resolution_m]
    axes[2].imshow(grid.cells,origin='lower',extent=extent,
                    cmap=ListedColormap(['#9499a1','#fafafa','#20252c']),
                    norm=BoundaryNorm([-1.5,-.5,.5,1.5],3))
    axes[2].set(xlabel='Local X (m)',ylabel='Local Y (m)',title='Tag center in NEW local floor map')
    for result in results:
        uv=np.array(result['corners_px']); closed=np.vstack((uv,uv[0]))
        for ax in axes[:2]:
            ax.plot(closed[:,0],closed[:,1],'r-',lw=2)
            ax.plot(*result['center_px'],'r+')
        axes[0].annotate(f"ID {result['tag_id']}",result['center_px'],xytext=(12,-15),
                          textcoords='offset points',color='red')
        if result['depth_valid']:
            x,y,z=result['center_xyz_m'];theta=result['theta_floor_parallel_rad']
            axes[2].plot(x,y,'ro')
            axes[2].arrow(x,y,.12*math.cos(theta),.12*math.sin(theta),
                           color='red',head_width=.03,length_includes_head=True)
            axes[2].annotate(f"ID {result['tag_id']}\n({x:.3f}, {y:.3f}, {z:.3f}) m",(x,y),
                             xytext=(8,8),textcoords='offset points',color='red')
    if results:
        uv=np.array(results[0]['corners_px'])
        axes[1].set_xlim(uv[:,0].min()-20,uv[:,0].max()+20)
        axes[1].set_ylim(uv[:,1].max()+20,uv[:,1].min()-20)
    fig.suptitle('Real AprilTag check — depth center; no independent ground truth / no robot motion')
    fig.tight_layout();fig.savefig(output/'tagcheck.png',dpi=150);plt.close(fig)
    return report
