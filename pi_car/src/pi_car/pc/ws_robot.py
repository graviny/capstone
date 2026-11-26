"""PC-side executor for a d455_drive/server.py vehicle (laptop-driven protocol).

The fleet-server/fleet-panel flow is unchanged: this process connects to the
fleet server exactly like ``robot-client`` does on the Pi, runs the same
RobotAgent (A*, waypoint follower, relative turn, safety) on the PC, and sends
the resulting left/right outputs as ``{"type":"wheels"}`` messages to the
car's WebSocket server. The car's ``status.dist_cm`` is the front range.

Fail-safe chain: the car stops on its own 0.5 s command timeout or on WebSocket
loss; on this side a lost/stale status makes the range stale, which stops the
agent (ULTRASONIC_INPUT_INVALID) and finishes the command.
"""
import asyncio
import contextlib
import json
import logging
import threading
import time

from pi_car.vehicle.perception import RangeReading

log = logging.getLogger(__name__)


class WebSocketWheels:
    """Motor driver for RobotAgent. Thread-safe: the watchdog thread calls stop()."""

    def __init__(self, loop, max_output):
        self.loop, self.max_output = loop, max_output
        self.lock = threading.Lock()
        self.latest = (0., 0.)
        self.wake = asyncio.Event()

    def _notify(self):
        with contextlib.suppress(RuntimeError):  # loop already closed at shutdown
            self.loop.call_soon_threadsafe(self.wake.set)

    def apply(self, command):
        clamp = lambda v: max(-self.max_output, min(self.max_output, v))
        with self.lock:
            self.latest = (clamp(command.left), clamp(command.right))
        self._notify()  # send at the agent's tick rate: latency matters for pivots

    def stop(self):
        with self.lock:
            self.latest = (0., 0.)
        self._notify()

    def outputs(self):
        with self.lock:
            return self.latest


class WebSocketCar:
    """One reconnecting WebSocket session to d455_drive/server.py --four-wheel."""

    def __init__(self, url, wheels, *, expected_robot_id=None, send_hz=20., reconnect_s=.5,
                 status_timeout_s=1.):
        self.url, self.wheels, self.expected = url, wheels, expected_robot_id
        self.period, self.reconnect, self.status_timeout = 1 / send_hz, reconnect_s, status_timeout_s
        self.range = None
        self.status = None
        self.running = True
        self.seq = 0

    def front_range(self):
        return self.range

    def _on_status(self, status):
        self.status = status
        dist = status.get("dist_cm")
        now = time.monotonic()
        self.range = (RangeReading(float(dist) / 100, 0., now)
                      if isinstance(dist, (int, float)) and dist > 0 else None)

    async def _session(self, ws):
        first = json.loads(await asyncio.wait_for(ws.recv(), self.status_timeout))
        if first.get("type") != "status":
            raise ValueError(f"Expected status first, got {first.get('type')}")
        if first.get("drive") != "differential":
            raise ValueError(f"Car drive is {first.get('drive')!r}; start server.py with --four-wheel")
        if self.expected and first.get("robot_id") != self.expected:
            raise ValueError(f"Car robot_id {first.get('robot_id')!r} != expected {self.expected!r}")
        self._on_status(first)
        print(f"[ws-robot] connected {self.url} robot={first.get('robot_id')} "
              f"max_speed={first.get('max_speed')} dist_cm={first.get('dist_cm')}", flush=True)

        async def reader():
            async for raw in ws:
                msg = json.loads(raw)
                if msg.get("type") == "status":
                    self._on_status(msg)
                elif msg.get("type") == "event":
                    print(f"[ws-robot] event {msg}", flush=True)
                elif msg.get("type") == "error":
                    raise ValueError(f"Car rejected command: {msg.get('error')}")

        async def writer():
            while True:
                with contextlib.suppress(asyncio.TimeoutError):
                    await asyncio.wait_for(self.wheels.wake.wait(), self.period)
                self.wheels.wake.clear()
                left, right = self.wheels.outputs()
                self.seq += 1
                await ws.send(json.dumps({"type": "wheels", "left": round(left * 100, 2),
                                          "right": round(right * 100, 2), "seq": self.seq}))

        async def watchdog():
            while True:
                await asyncio.sleep(self.period)
                if self.range is not None and time.monotonic() - self.range.received_monotonic > self.status_timeout:
                    raise TimeoutError("Car status stopped arriving")

        tasks = [asyncio.create_task(c) for c in (reader(), writer(), watchdog())]
        try:
            done, _ = await asyncio.wait(tasks, return_when=asyncio.FIRST_COMPLETED)
            for task in done:
                task.result()
        finally:
            for task in tasks:
                task.cancel()
            await asyncio.gather(*tasks, return_exceptions=True)
            with contextlib.suppress(Exception):
                await ws.send(json.dumps({"type": "stop"}))

    async def run(self):
        from websockets.asyncio.client import connect
        while self.running:
            try:
                async with connect(self.url, open_timeout=2, ping_interval=1, ping_timeout=2) as ws:
                    await self._session(ws)
            except asyncio.CancelledError:
                raise
            except Exception as error:  # noqa: BLE001 - any failure → stopped car, reconnect
                log.warning("Car session %s: %s", self.url, error)
                print(f"[ws-robot] car link down: {error}", flush=True)
            finally:
                self.range = None
                self.wheels.stop()
            if self.running:
                await asyncio.sleep(self.reconnect)
