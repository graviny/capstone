"""Fleet CLI adapters, kept separate from the mapping/inspection CLI."""
import asyncio
import json
from pathlib import Path
import time
from contextlib import nullcontext


def add_commands(commands):
    server = commands.add_parser("fleet-server", help="PC TCP server + multi-AprilTag localization")
    server.add_argument("--map", required=True)
    server.add_argument("--calibration", required=True)
    server.add_argument("--robots", required=True)
    server.add_argument("--host", default="0.0.0.0")
    server.add_argument("--port", type=int, default=8765)
    server.add_argument("--command-port", type=int, default=8766)
    server.add_argument("--clearance", type=float, default=.2)
    server.add_argument("--journal", default="data/fleet.sqlite3")
    server.add_argument("--snapshot-dir",help="Optional live RGB-D diagnostic snapshot directory (1 Hz)")
    server.add_argument("--minimum-pose-age-ms", type=float, default=0.,
                        help="Delay fresh pose publication to accommodate measured Pi clock lag")
    robot = commands.add_parser("robot-client", help="Pi executor; dry-run unless --enable-motors is explicit")
    robot.add_argument("--map", required=True)
    robot.add_argument("--robot-id", required=True)
    robot.add_argument("--host", required=True)
    robot.add_argument("--port", type=int, default=8765)
    robot.add_argument("--clearance", type=float, default=.2)
    robot.add_argument("--journal", required=True, help="Unique durable SQLite file for this robot")
    robot.add_argument("--mock-range", type=float, help="DRY-RUN synthetic front range in meters")
    robot.add_argument("--enable-motors",action="store_true",help="Activate real Adeept motor and ultrasonic hardware")
    robot.add_argument("--allow-no-echo",action="store_true",
                       help="Treat repeated fresh 2m ultrasonic saturation as clear; still stop for measured obstacles")
    robot.add_argument("--vendor-dir",help="Installed Adeept OrdinaryWheels directory containing Move.py")
    robot.add_argument("--max-output",type=float,default=.2)
    robot.add_argument("--drive-output",type=float,
                       help="Cruise motor output (0..1), not m/s; default 0.20 capped by --max-output")
    robot.add_argument("--servo-angle",type=float,default=90.)
    panel=commands.add_parser("fleet-panel",help="PC map window: choose robot, click goal, STOP")
    panel_map=panel.add_mutually_exclusive_group(required=True)
    panel_map.add_argument("--map")
    panel_map.add_argument("--mock-map",action="store_true")
    panel.add_argument("--host",default="127.0.0.1")
    panel.add_argument("--port",type=int,default=8766)
    panel.add_argument("--clearance",type=float,default=.2)
    panel.add_argument("--camera-dir",help="Show live fleet-server RGB snapshots and click the floor image")
    panel.add_argument("--calibration",help="Camera calibration for --camera-dir pixel goals")
    sender = commands.add_parser("send-command", help="Send an external JSON command; no auto goals")
    sender.add_argument("--host", default="127.0.0.1")
    sender.add_argument("--port", type=int, default=8766)
    sender.add_argument("--json", required=True)
    mock = commands.add_parser("mock-fleet", help="2 dry robots, mock external poses, waits for commands")
    mock.add_argument("--port", type=int, default=8765)
    mock.add_argument("--command-port", type=int, default=8766)


async def run(args):
    from pi_car.common.grid import OccupancyGrid
    from pi_car.pc.commands import CommandInterface
    from pi_car.transport.commands import CommandTCPServer, send_command
    if args.command == "fleet-panel":
        from pi_car.pc.panel import FleetPanel
        from pi_car.pc.calibration import Calibration
        from pi_car.mock import mock_grid
        if bool(args.camera_dir) != bool(args.calibration):
            raise ValueError('--camera-dir and --calibration must be used together')
        grid=mock_grid() if args.mock_map else OccupancyGrid.load(args.map)
        calibration=Calibration.load(args.calibration) if args.calibration else None
        if calibration and calibration.frame_id != grid.metadata.frame_id:
            raise ValueError('Camera calibration and fleet map frames differ')
        FleetPanel(grid,host=args.host,port=args.port,clearance_m=args.clearance,
                   camera_dir=args.camera_dir,calibration=calibration).show()
        return
    if args.command == "send-command":
        print(json.dumps(await send_command(args.host,args.port,json.loads(args.json)),indent=2))
        return
    if args.command == "mock-fleet":
        from pi_car.mock import MockFleet
        world = await MockFleet(port=args.port).start()
        gateway = CommandTCPServer(CommandInterface(world.fleet).handle,port=args.command_port)
        try:
            await gateway.start()
            print(f"Mock robots A/B await external commands at 127.0.0.1:{gateway.port}",flush=True)
            await asyncio.Event().wait()
        finally:
            await gateway.close()
            await world.close()
        return
    if args.command == "robot-client":
        from pi_car.vehicle.agent import RobotAgent
        from pi_car.vehicle.client import RobotClient
        from pi_car.vehicle.perception import RangeReading
        from pi_car.transport.protocol import number
        if args.mock_range is not None and number(args.mock_range) <= 0:
            raise ValueError("Mock range must be positive")
        if args.enable_motors and (not args.vendor_dir or args.mock_range is not None):
            raise ValueError('--enable-motors requires --vendor-dir and forbids --mock-range')
        if args.allow_no_echo and not args.enable_motors:
            raise ValueError('--allow-no-echo requires --enable-motors')
        drive_output=(number(args.drive_output) if args.drive_output is not None else
                      min(.2,number(args.max_output)) if args.enable_motors else .2)
        if not 0 < drive_output <= 1:
            raise ValueError('--drive-output must be within (0, 1]')
        if args.enable_motors and not 0 < drive_output <= number(args.max_output) <= .3:
            raise ValueError('Hardware requires 0 < --drive-output <= --max-output <= 0.30')
        grid=OccupancyGrid.load(args.map)
        if args.enable_motors:
            from pi_car.vehicle.hardware.live import AdeeptHardware
            context=AdeeptHardware(args.vendor_dir,max_output=args.max_output,
                                   servo_angle=args.servo_angle)
        else:context=nullcontext(None)
        with context as hardware:
            agent = RobotAgent(args.robot_id,grid,clearance_m=args.clearance,
                               journal_path=args.journal,motor=hardware.motor if hardware else None,
                               drive_output=drive_output)
            provider=(hardware.sensor.read_allow_clear if hardware and args.allow_no_echo else
                      hardware.sensor.read if hardware else
                      (lambda:RangeReading(args.mock_range,0.,time.monotonic())) if args.mock_range else None)
            client = RobotClient(agent,args.host,args.port,range_provider=provider)
            try:
                label=('REAL HARDWARE: fresh no-echo means clear; measured obstacles still stop' if args.allow_no_echo else
                       'REAL HARDWARE ENABLED' if hardware else 'DRY RUN: motor output disabled')
                print(label,flush=True)
                await client.run()
            finally:
                await client.close()
                await agent.close()
        return
    from pi_car.pc.calibration import Calibration
    from pi_car.pc.camera import RealSenseCamera
    from pi_car.pc.fleet import FleetController
    from pi_car.pc.localization import MultiRobotLocalizer
    from pi_car.pc.server import FleetTCPServer
    from pi_car.pc.live import publish_camera
    minimum_pose_age_s = args.minimum_pose_age_ms / 1000
    if not 0 <= minimum_pose_age_s < .3:
        raise ValueError('Minimum pose age must be in [0, 300) milliseconds')
    grid = OccupancyGrid.load(args.map)
    calibration = Calibration.load(args.calibration)
    if calibration.frame_id != grid.metadata.frame_id:
        raise ValueError("Calibration and map frames differ")
    configs = json.loads(Path(args.robots).read_text())["robots"]
    localizer = MultiRobotLocalizer(calibration,configs)
    fleet = FleetController(grid,list(localizer.localizers),clearance_m=args.clearance,
                             journal_path=args.journal)
    server = FleetTCPServer(fleet,args.host,args.port)
    gateway = CommandTCPServer(CommandInterface(fleet).handle,port=args.command_port)
    try:
        await server.start(); await gateway.start()
        with RealSenseCamera() as camera:
            print(f"Fleet TCP {server.host}:{server.port}; command API 127.0.0.1:{gateway.port}",flush=True)
            await publish_camera(fleet,camera,localizer,snapshot_dir=args.snapshot_dir,
                                 minimum_pose_age_s=minimum_pose_age_s)
    finally:
        await gateway.close(); await server.close(); fleet.close()
