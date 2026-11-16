"""D455 → one multi-tag detection → independent poses; never computes paths."""
import asyncio


async def publish_camera(fleet, camera, localizer):
    def observe():
        frame = camera.read()
        return localizer.estimate(frame)

    while True:
        try:
            poses = await asyncio.to_thread(observe)
        except (ValueError, RuntimeError):
            poses = {robot_id: None for robot_id in fleet.robots}
            await asyncio.sleep(.05)
        await asyncio.gather(*(fleet.publish_pose(r, poses.get(r)) for r in fleet.robots))
