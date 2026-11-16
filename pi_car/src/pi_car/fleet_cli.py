"""Fleet CLI adapters, kept separate from the mapping/inspection CLI."""
import asyncio
import json
from pathlib import Path
import time


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
    robot = commands.add_parser("robot-client", help="Pi executor; motors always disabled in this CLI")
    robot.add_argument("--map", required=True)
    robot.add_argument("--robot-id", required=True)
    robot.add_argument("--host", required=True)
    robot.add_argument("--port", type=int, default=8765)
    robot.add_argument("--clearance", type=float, default=.2)
    robot.add_argument("--journal", required=True, help="Unique durable SQLite file for this robot")
    robot.add_argument("--mock-range", type=float, help="DRY-RUN synthetic front range in meters")
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
        agent = RobotAgent(args.robot_id,OccupancyGrid.load(args.map),
                           clearance_m=args.clearance,journal_path=args.journal)
        provider = (lambda: RangeReading(args.mock_range,0.,time.monotonic())) if args.mock_range else None
        client = RobotClient(agent,args.host,args.port,range_provider=provider)
        try:
            print("DRY RUN: motor output disabled; real pose input still required",flush=True)
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
            await publish_camera(fleet,camera,localizer)
    finally:
        await gateway.close(); await server.close(); fleet.close()
