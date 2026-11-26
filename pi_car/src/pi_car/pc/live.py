"""D455 → one multi-tag detection → independent poses; never computes paths."""
import asyncio
from pathlib import Path
import time


async def publish_camera(fleet, camera, localizer, *, snapshot_dir=None, minimum_pose_age_s=0.):
    last_snapshot=0.
    directory=Path(snapshot_dir) if snapshot_dir else None
    if directory:directory.mkdir(parents=True,exist_ok=True)
    def observe():
        nonlocal last_snapshot
        frame = camera.read()
        poses=localizer.estimate(frame)
        if directory and time.monotonic()-last_snapshot>=.25:
            temporary=directory/'latest.tmp.npz'
            frame.save(temporary)
            temporary.replace(directory/'latest.npz')
            last_snapshot=time.monotonic()
        return poses

    while True:
        error=None
        try:
            poses = await asyncio.to_thread(observe)
        except (ValueError, RuntimeError) as exception:
            error=str(exception)
            poses = {robot_id: None for robot_id in fleet.robots}
            await asyncio.sleep(.05)
        captured_at = max((pose.timestamp for pose in poses.values() if pose is not None), default=None)
        if captured_at is not None and minimum_pose_age_s:
            delay = captured_at + minimum_pose_age_s - time.time()
            if delay > 0:
                await asyncio.sleep(delay)
        for r in fleet.robots:
            if error:
                reason=error
            elif poses.get(r) is not None:
                reason=None
            elif (localizer.localizers[r].tag_family,localizer.localizers[r].tag_id) not in localizer.last_detection_keys:
                reason=f"TAG {localizer.localizers[r].tag_family}:{localizer.localizers[r].tag_id} NOT DETECTED (seen: {sorted(localizer.last_detection_keys)})"
            else:
                reason="TAG POSE REJECTED: check size, view angle and reprojection error"
            fleet.robots[r].pose_error=reason
        await asyncio.gather(*(fleet.publish_pose(r, poses.get(r)) for r in fleet.robots))
