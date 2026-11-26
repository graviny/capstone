"""Mapping, fleet, manual goals and opt-in Adeept driving commands."""
import argparse
from dataclasses import asdict
from pathlib import Path
import json
import asyncio
from contextlib import nullcontext


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    from pi_car.fleet_cli import add_commands
    add_commands(commands)
    capture = commands.add_parser("capture", help="Save one rectified RGB-D capture")
    capture.add_argument("--out", required=True)
    capture.add_argument("--frames", type=int, default=1,
                         help="Combine successive depth frames for stationary mapping (default: 1)")
    preview = commands.add_parser("preview", help="Check RGB/depth before calibration")
    preview_source = preview.add_mutually_exclusive_group(required=True)
    preview_source.add_argument("--capture")
    preview_source.add_argument("--live", action="store_true")
    preview.add_argument("--out", default="data/hardware_check")
    preview.add_argument("--show", action="store_true", help="Open saved capture in a GUI window")
    experiment = commands.add_parser("experiment", help="Depth floor fit → map → test goal/path report")
    experiment.add_argument("--capture", required=True)
    floor_source=experiment.add_mutually_exclusive_group(required=True)
    floor_source.add_argument("--floor-roi", type=int, nargs=4, metavar=("X0","Y0","X1","Y1"))
    floor_source.add_argument("--pick-floor", action="store_true",help="Click two corners of a clear floor patch")
    experiment.add_argument("--out", required=True)
    experiment.add_argument("--clearance", type=float, default=.2)
    experiment.add_argument("--resolution", type=float, default=.03)
    experiment.add_argument("--robot-config")
    experiment.add_argument("--floor-model", choices=("plane", "quadratic"), default="quadratic",
                            help="quadratic removes the D455 depth bowl toward the image edges")
    experiment.add_argument("--obstacle-fraction", type=float, default=.25,
                            help="Share of a cell's depth samples that must be raised to mark it occupied")
    experiment.add_argument("--boundary-tags", type=int, nargs="+", metavar="ID",
                            help="ArUco 4x4 tag IDs at the area corners; outside their polygon is not drivable")
    experiment.add_argument("--boundary-px", type=float, nargs=2, action="append", metavar=("U","V"),
                            help="Extra corner pixel for a tag clipped by the image edge (repeatable)")
    experiment.add_argument("--floor-below", type=float, default=.06,
                            help="Depth this far below the floor still counts as floor (m)")
    tagcheck = commands.add_parser("tagcheck", help="Detect tags and locate centers using captured depth")
    tagcheck.add_argument("--capture", required=True)
    tagcheck.add_argument("--calibration", required=True)
    tagcheck.add_argument("--map", required=True)
    tagcheck.add_argument("--out", required=True)
    tagcheck.add_argument("--tag-size", type=float, help="Optional MEASURED black square edge in meters")
    calibrate = commands.add_parser("calibrate", help="Click measured floor references in order")
    calibrate.add_argument("--capture", required=True)
    calibrate.add_argument("--references", required=True, help="JSON list of measured world XY points")
    calibrate.add_argument("--out", required=True)
    calibrate.add_argument("--max-rmse", type=float, default=.03)
    mapping = commands.add_parser("map", help="Static capture → calibrated occupancy grid")
    mapping.add_argument("--capture", required=True)
    mapping.add_argument("--calibration", required=True)
    mapping.add_argument("--config", required=True)
    mapping.add_argument("--out", required=True)
    inspect = commands.add_parser("inspect", help="View map, inspect pose, click goal, preview A*")
    inspect.add_argument("--map", required=True)
    inspect.add_argument("--calibration")
    source = inspect.add_mutually_exclusive_group()
    source.add_argument("--capture")
    source.add_argument("--live", action="store_true")
    inspect.add_argument("--robot-config", default=None)
    inspect.add_argument("--clearance", type=float, default=.20,
                         help="Car circumscribed radius + margin, meters; measure your car")
    inspect.add_argument("--out", default="data/inspection")
    args = parser.parse_args()
    try:
        _run(args, parser)
    except (ValueError, RuntimeError, OSError, ImportError, KeyError) as error:
        parser.exit(1, f"Error: {error}\n")


def _run(args, parser):
    if args.command in {"fleet-server", "robot-client", "send-command", "mock-fleet", "fleet-panel", "ws-robot"}:
        from pi_car.fleet_cli import run
        try:
            asyncio.run(run(args))
        except KeyboardInterrupt:
            pass
        return
    from pi_car.pc.camera import RealSenseCamera, RGBDFrame, capture_static
    from pi_car.pc.calibration import Calibration, calibrate
    if args.command == "capture":
        if args.frames < 1:
            raise ValueError("--frames must be positive")
        with RealSenseCamera() as camera:
            frame = capture_static(camera, args.frames)
        frame.save(args.out)
        print(f"Saved {args.out}; {frame.frame_count} frame(s), rectified depth in meters")
    elif args.command == "tagcheck":
        from pi_car.common.grid import OccupancyGrid
        from pi_car.pc.tagcheck import check_tags
        report = check_tags(RGBDFrame.load(args.capture), Calibration.load(args.calibration),
                            OccupancyGrid.load(args.map), args.out, args.tag_size)
        print(json.dumps(report, indent=2))
    elif args.command == "experiment":
        from pi_car.pc.experiment import run_experiment
        from pi_car.pc.floor import pick_floor_roi
        robot = json.loads(Path(args.robot_config).read_text()) if args.robot_config else None
        frame=RGBDFrame.load(args.capture)
        roi=pick_floor_roi(frame) if args.pick_floor else args.floor_roi
        boundary=None
        if args.boundary_tags or args.boundary_px:
            from pi_car.pc.floor import find_tag_centers
            boundary=(find_tag_centers(frame,args.boundary_tags) if args.boundary_tags else [])+(args.boundary_px or [])
        report = run_experiment(frame, roi, args.out,
                                clearance_m=args.clearance, resolution_m=args.resolution,
                                robot_config=robot, floor_model=args.floor_model,
                                obstacle_fraction=args.obstacle_fraction,
                                floor_below_m=args.floor_below, boundary_px=boundary)
        print(json.dumps(report, indent=2))
    elif args.command == "preview":
        from pi_car.pc.preview import RGBDPreview
        preview = RGBDPreview()
        if args.live:
            with RealSenseCamera() as camera:
                def refresh():
                    try:
                        preview.update(camera.read())
                    except (RuntimeError, ValueError) as error:
                        preview.axes[0].set_title(f"Camera error: {error}")
                        preview.fig.canvas.draw_idle()
                timer = preview.fig.canvas.new_timer(interval=100)
                timer.add_callback(refresh)
                timer.start()
                preview.plt.show()
                timer.stop()
        else:
            stats = preview.save(RGBDFrame.load(args.capture), args.out)
            print(json.dumps(stats, indent=2))
            if args.show:
                preview.plt.show()
            else:
                preview.plt.close(preview.fig)
    elif args.command == "calibrate":
        import matplotlib.pyplot as plt
        frame = RGBDFrame.load(args.capture)
        references = json.loads(Path(args.references).read_text())
        if len(references) < 4:
            raise ValueError("At least four measured floor references are required")
        fig, ax = plt.subplots()
        ax.imshow(frame.rgb)
        ax.set_title(f"Click {len(references)} floor reference points IN ORDER; close to cancel")
        print("Click pixels corresponding to:", references)
        pixels = plt.ginput(len(references), timeout=0)
        plt.close(fig)
        result = calibrate(frame, pixels, references, args.max_rmse)
        result.save(args.out)
        Path(args.out).with_suffix(".references.json").write_text(
            json.dumps(dict(pixels=pixels, world_xy=references, capture=args.capture), indent=2))
        print(f"Calibration RMSE={result.rmse_m:.4f}m; frame_id={result.frame_id}")
    elif args.command == "map":
        from pi_car.pc.mapping import build_grid
        frame, calibration = RGBDFrame.load(args.capture), Calibration.load(args.calibration)
        config = json.loads(Path(args.config).read_text())
        grid = build_grid(frame, calibration, **config)
        grid.save(args.out)
        Path(args.out, "mapping_config.json").write_text(json.dumps(config, indent=2))
        Path(args.out, "calibration.json").write_text(json.dumps(asdict(calibration), indent=2))
        print(f"Saved {grid.metadata.width}x{grid.metadata.height} map to {args.out}")
    else:
        from pi_car.common.grid import OccupancyGrid
        from pi_car.pc.viewer import MapInspector
        if (args.live or args.capture) and (not args.calibration or not args.robot_config):
            parser.error("--live/--capture requires --calibration and --robot-config")
        grid = OccupancyGrid.load(args.map)
        viewer = MapInspector(grid, args.clearance, args.out)
        if args.live or args.capture:
            from pi_car.pc.localization import AprilTagLocalizer
            calibration = Calibration.load(args.calibration)
            if calibration.frame_id != grid.metadata.frame_id:
                raise ValueError("Map/calibration frame IDs differ")
            robot = json.loads(Path(args.robot_config).read_text())
            localizer = AprilTagLocalizer(calibration, **robot)
            with RealSenseCamera() if args.live else nullcontext(None) as camera:
                if camera:
                    # Use a GUI timer, not a blocking while loop, so clicks remain responsive.
                    def refresh():
                        try:
                            frame = camera.read()
                            viewer.update(frame, calibration, localizer.estimate(frame))
                        except (RuntimeError, ValueError) as error:
                            viewer.pose = None
                            viewer.pose_artist.set_data([], [])
                            viewer.heading_artist.set_data([], [])
                            if not viewer.manual_start:
                                viewer.start = None
                            viewer._replan()
                            Path(args.out, "pose.json").write_text(json.dumps(dict(valid=False)))
                            viewer._message(f"Camera/localization error: {error}")
                    timer = viewer.fig.canvas.new_timer(interval=100)
                    timer.add_callback(refresh)
                    timer.start()
                    viewer.plt.show()
                    timer.stop()
                else:
                    frame = RGBDFrame.load(args.capture)
                    viewer.update(frame, calibration, localizer.estimate(frame))
                    viewer.plt.show()
        else:
            viewer.plt.show()


if __name__ == "__main__":
    main()
