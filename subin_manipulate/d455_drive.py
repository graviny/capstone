#!/usr/bin/env python3
"""Use D455 depth to choose local driving commands for the PiCar-B2."""

import argparse
from pathlib import Path
import signal
import sys
import time

import numpy as np


def sector_distances(depth, scale):
    height, width = depth.shape
    band = depth[height * 40 // 100 : height * 75 // 100, width // 8 : width * 7 // 8]
    distances = []
    for sector in np.array_split(band, 3, axis=1):
        valid = sector[sector > 0]
        if valid.size < 0.65 * sector.size:
            distances.append(None)
        else:
            distances.append(float(np.percentile(valid, 5)) * scale)
    return tuple(distances)


def choose_direction(distances, stop_m, avoid_m):
    if any(distance is None for distance in distances):
        return "stop"

    left, center, right = distances
    if min(left, center, right) <= stop_m:
        return "stop"
    if center >= avoid_m:
        return "forward"
    if max(left, right) < avoid_m + 0.20:
        return "stop"
    return "left" if left >= right else "right"


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--drive", action="store_true", help="Enable motor and steering outputs")
    parser.add_argument("--speed", type=int, default=20, help="Forward drive PWM percent (1–35)")
    parser.add_argument("--steering", type=int, default=25, help="Steering angle from center (1–30)")
    parser.add_argument("--stop-m", type=float, default=0.8, help="Stop distance in meters")
    parser.add_argument("--avoid-m", type=float, default=1.3, help="Start avoiding below this distance")
    args = parser.parse_args()
    if not 1 <= args.speed <= 35 or not 1 <= args.steering <= 30:
        parser.error("--speed must be 1–35 and --steering must be 1–30")
    if not 0.6 <= args.stop_m < args.avoid_m < float("inf"):
        parser.error("require 0.6 <= --stop-m < --avoid-m, both finite")
    return args


def run(args):
    import pyrealsense2 as rs

    context = rs.context()
    cameras = [device for device in context.query_devices()
               if "D455" in device.get_info(rs.camera_info.name)]
    if len(cameras) != 1:
        raise RuntimeError("Connect exactly one RealSense D455 over USB 3.")

    pipeline = rs.pipeline(context)
    config = rs.config()
    config.enable_device(cameras[0].get_info(rs.camera_info.serial_number))
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    profile = pipeline.start(config)
    motor_controller = None
    try:
        scale = profile.get_device().first_depth_sensor().get_depth_scale()
        if args.drive:
            sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "web"))
            import move
            from RPIservo import ServoCtrl

            move.setup()
            motor_controller = move
            move.motorStop()
            steering = ServoCtrl()

        angles = {"forward": 0, "left": args.steering, "right": -args.steering}
        steer_angle = None
        steer_ready_at = 0.0
        last_frame = None
        last_log = 0.0
        warmup_until = time.monotonic() + 1.0
        while True:
            ok, frames = pipeline.try_wait_for_frames(250)
            depth = frames.get_depth_frame() if ok else None
            distances = (None, None, None)
            if depth and depth.get_frame_number() != last_frame:
                last_frame = depth.get_frame_number()
                distances = sector_distances(np.asanyarray(depth.get_data()), scale)

            now = time.monotonic()
            command = choose_direction(distances, args.stop_m, args.avoid_m)
            if now < warmup_until:
                command = "stop"
            applied = command
            if motor_controller is not None:
                if command == "stop":
                    move.motorStop()
                elif angles[command] != steer_angle:
                    move.motorStop()
                    steering.moveAngle(0, angles[command])
                    steer_angle = angles[command]
                    steer_ready_at = time.monotonic() + 0.25
                    applied = "stop (steering)"
                elif now >= steer_ready_at:
                    move.move(args.speed, 1, "no")
                else:
                    applied = "stop (steering)"

            if now - last_log >= 0.5:
                readings = " ".join(
                    f"{side}={'--' if value is None else f'{value:.2f}m'}"
                    for side, value in zip(("L", "C", "R"), distances)
                )
                print(f"{readings} decision={command} {'output=' + applied if args.drive else '(preview)'}",
                      flush=True)
                last_log = now
    finally:
        try:
            if motor_controller is not None:
                motor_controller.destroy()
        finally:
            pipeline.stop()


def interrupt(*_):
    raise KeyboardInterrupt


def main():
    args = parse_args()
    signal.signal(signal.SIGTERM, interrupt)
    try:
        run(args)
    except KeyboardInterrupt:
        return 0
    except (ImportError, OSError, RuntimeError, ValueError) as exc:
        print(f"D455 drive: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
