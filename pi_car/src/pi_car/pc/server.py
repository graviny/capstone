"""Multi-client TCP adapter for FleetController; contains no path planner."""
import asyncio
import time
from pi_car.transport.protocol import MAX_MESSAGE_BYTES, STATUS
from pi_car.transport.tcp import JsonConnection


class FleetTCPServer:
    def __init__(self, fleet, host="127.0.0.1", port=8765, *, timeout_s=2., heartbeat_s=.5):
        self.fleet, self.host, self.port = fleet, host, port
        self.timeout, self.heartbeat = timeout_s, heartbeat_s
        self.server = None
        self.tasks, self.connections = set(), set()

    async def start(self):
        self.server = await asyncio.start_server(self._session, self.host, self.port,
                                                  limit=MAX_MESSAGE_BYTES)
        self.port = self.server.sockets[0].getsockname()[1]
        return self

    async def _ping(self, connection, robot_id):
        while True:
            await asyncio.sleep(self.heartbeat)
            await connection.send(self.fleet.message("PING", robot_id))

    async def _session(self, reader, writer):
        task = asyncio.current_task()
        self.tasks.add(task)
        connection = JsonConnection(reader, writer, self.timeout)
        self.connections.add(connection)
        robot_id = None
        pinger = None
        try:
            hello = await connection.receive()
            if hello["type"] != "HELLO":
                raise ValueError("HELLO must be first")
            self.fleet.register(hello, connection)
            robot_id = hello["robot_id"]
            pinger = asyncio.create_task(self._ping(connection, robot_id))
            while True:
                event = await connection.receive()
                if (self.fleet.robots[robot_id].sender is not connection
                        or event["robot_id"] != robot_id or event["frame_id"] != hello["frame_id"]
                        or event["map_version"] != hello["map_version"]):
                    raise ValueError("Session identity/map mismatch")
                self.fleet.robots[robot_id].last_communication = time.time()
                if event["type"] == "PING":
                    await connection.send(self.fleet.message("PONG", robot_id))
                elif event["type"] == "PONG":
                    pass
                elif event["type"] in STATUS:
                    self.fleet.status_received(event)
                else:
                    raise ValueError("Unexpected Pi message")
        except (OSError, ValueError, ConnectionError, asyncio.TimeoutError):
            pass
        finally:
            if pinger:
                pinger.cancel()
                await asyncio.gather(pinger, return_exceptions=True)
            if robot_id:
                self.fleet.disconnected(robot_id, connection)
            await connection.close()
            self.connections.discard(connection)
            self.tasks.discard(task)

    async def close(self):
        if self.server:
            self.server.close()
            await self.server.wait_closed()
        for connection in list(self.connections):
            await connection.close()
        for task in list(self.tasks):
            task.cancel()
        await asyncio.gather(*self.tasks, return_exceptions=True)
