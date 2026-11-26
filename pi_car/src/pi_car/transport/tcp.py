import asyncio
from .protocol import MAX_MESSAGE_BYTES, encode_message, decode_message


class JsonConnection:
    """Exactly one reader; concurrent sends serialized; bounded frame and drain time."""
    def __init__(self, reader, writer, timeout_s=2.):
        self.reader, self.writer, self.timeout = reader, writer, timeout_s
        self._send_lock = asyncio.Lock()

    async def receive(self):
        try:
            raw = await asyncio.wait_for(self.reader.readline(), self.timeout)
        except (ValueError, asyncio.LimitOverrunError) as error:
            raise ValueError("TCP frame exceeds limit") from error
        if not raw:
            raise ConnectionError("Peer disconnected")
        return decode_message(raw)

    async def send(self, message):
        raw = encode_message(message)
        async with self._send_lock:
            if self.writer.is_closing():
                raise ConnectionError("Connection closed")
            self.writer.write(raw)
            await asyncio.wait_for(self.writer.drain(), self.timeout)

    async def close(self):
        self.writer.close()
        try:
            await asyncio.wait_for(self.writer.wait_closed(), self.timeout)
        except (OSError, asyncio.TimeoutError):
            pass


async def open_connection(host, port, timeout_s=2.):
    reader, writer = await asyncio.wait_for(
        asyncio.open_connection(host, port, limit=MAX_MESSAGE_BYTES), timeout_s)
    return JsonConnection(reader, writer, timeout_s)
