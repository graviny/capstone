import asyncio
import contextlib
from pi_car.transport.tcp import open_connection


class RobotClient:
    def __init__(self, agent, host="127.0.0.1", port=8765, *, tick_s=.02,
                 heartbeat_s=.5, timeout_s=2., reconnect_s=.5, range_provider=None):
        self.agent, self.host, self.port = agent, host, port
        self.tick_s, self.heartbeat, self.timeout, self.reconnect = tick_s, heartbeat_s, timeout_s, reconnect_s
        self.range_provider = range_provider
        self.running = True
        self.connection = None

    async def _reader(self, connection):
        while True:
            event = await connection.receive()
            if (event["robot_id"] != self.agent.robot_id
                    or event["frame_id"] != self.agent.grid.metadata.frame_id
                    or event["map_version"] != self.agent.grid.metadata.version):
                raise ValueError("Session identity/map mismatch")
            if event["type"] == "PING":
                await connection.send(self.agent.message("PONG"))
            elif event["type"] == "PONG":
                pass
            else:
                await self.agent.handle(event)

    async def _writer(self, connection, queue):
        while True:
            await connection.send(await queue.get())

    async def _pinger(self, connection):
        while True:
            await asyncio.sleep(self.heartbeat)
            await connection.send(self.agent.message("PING"))

    async def _control(self):
        while True:
            if self.range_provider is not None:
                self.agent.front_range = self.range_provider()
            self.agent.tick()
            await asyncio.sleep(self.tick_s)

    async def run(self):
        try:
            while self.running:
                tasks = []
                try:
                    connection = await open_connection(self.host, self.port, self.timeout)
                    self.connection = connection
                    queue = asyncio.Queue(maxsize=1024)
                    self.agent.emit = queue.put_nowait
                    await connection.send(self.agent.message("HELLO", clearance_m=self.agent.clearance))
                    self.agent.connect()
                    tasks = [asyncio.create_task(c) for c in (
                        self._reader(connection), self._writer(connection, queue),
                        self._pinger(connection), self._control())]
                    await asyncio.wait(tasks, return_when=asyncio.FIRST_COMPLETED)
                except (OSError, ValueError, ConnectionError, asyncio.TimeoutError):
                    pass
                finally:
                    # Halt before any await. Reconnection clears pose and never resumes.
                    self.agent.emit = lambda event: None
                    self.agent.disconnect()
                    for task in tasks:
                        task.cancel()
                    await asyncio.gather(*tasks, return_exceptions=True)
                    if self.connection:
                        await self.connection.close()
                        self.connection = None
                if self.running:
                    await asyncio.sleep(self.reconnect)
        finally:
            self.agent.emit = lambda event: None
            self.agent.disconnect()

    async def close(self):
        self.running = False
        self.agent.emit = lambda event: None
        self.agent.disconnect()
        if self.connection:
            await self.connection.close()
