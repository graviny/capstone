"""Small loopback JSON command adapter for testing the external interface."""
import asyncio
import json
from .protocol import MAX_MESSAGE_BYTES


class CommandTCPServer:
    def __init__(self, handler, host="127.0.0.1", port=8766):
        self.handler, self.host, self.port = handler, host, port
        self.server = None
        self.tasks = set()

    async def start(self):
        self.server = await asyncio.start_server(self._request, self.host, self.port,
                                                  limit=MAX_MESSAGE_BYTES)
        self.port = self.server.sockets[0].getsockname()[1]
        return self

    async def _request(self, reader, writer):
        task = asyncio.current_task()
        self.tasks.add(task)
        try:
            try:
                raw = await asyncio.wait_for(reader.readline(), 3.)
                if len(raw) > MAX_MESSAGE_BYTES or not raw.endswith(b"\n"):
                    raise ValueError("Invalid command framing")
                result = await self.handler(json.loads(raw))
                response = dict(ok=True, result=result)
            except (ValueError, TypeError, KeyError, asyncio.TimeoutError) as error:
                response = dict(ok=False, error=str(error))
            writer.write((json.dumps(response, allow_nan=False)+"\n").encode())
            await asyncio.wait_for(writer.drain(), 3.)
        except (OSError, ConnectionError):
            pass
        finally:
            writer.close()
            try:
                await writer.wait_closed()
            except OSError:
                pass
            self.tasks.discard(task)

    async def close(self):
        if self.server:
            self.server.close()
            await self.server.wait_closed()
        tasks = list(self.tasks)
        for task in tasks:
            task.cancel()
        await asyncio.gather(*tasks, return_exceptions=True)


async def send_command(host, port, request):
    reader, writer = await asyncio.wait_for(asyncio.open_connection(host, port), 3.)
    try:
        raw = (json.dumps(request, allow_nan=False)+"\n").encode()
        if len(raw) > MAX_MESSAGE_BYTES:
            raise ValueError("Command too large")
        writer.write(raw)
        await asyncio.wait_for(writer.drain(), 3.)
        return json.loads(await asyncio.wait_for(reader.readline(), 5.))
    finally:
        writer.close()
        await writer.wait_closed()
