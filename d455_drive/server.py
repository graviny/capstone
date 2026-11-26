#!/usr/bin/env python3
"""노트북에서 속도/조향 명령을 받아 PiCar를 움직이는 WebSocket 서버.

    sudo python3 server.py              # 실차
    python3 server.py --dry-run -v      # 하드웨어 없이 시험

프로토콜은 README.md 참고.
"""
import argparse
import asyncio
import json
import signal
import time

from websockets.asyncio.server import serve
from websockets.exceptions import ConnectionClosed

import config as C
from hardware import Car, FakeCar
from safety import Safety


class Controller:
    def __init__(self, car, max_speed):
        self.car = car
        self.safety = Safety(car)
        self.max_speed = max_speed
        self.cmd_speed = 0.0
        self.cmd_steer = 0.0
        self.last_cmd_t = 0.0
        self.last_seq = None
        self.client = None
        self.out_speed = 0.0
        self.out_steer = 0.0
        self.watchdog = True
        self.battery = None

    # ---- 수신 ----
    def handle(self, msg):
        t = msg.get("type")
        if t == "cmd":
            sp = float(msg.get("speed", 0))
            st = float(msg.get("steer", 0))
            self.cmd_speed = max(-self.max_speed, min(self.max_speed, sp))
            self.cmd_steer = max(-C.STEER_MAX_DEG, min(C.STEER_MAX_DEG, st))
            self.last_cmd_t = time.monotonic()
            self.last_seq = msg.get("seq")
            return None
        if t == "stop":
            self.cmd_speed = self.cmd_steer = 0.0
            self.last_cmd_t = time.monotonic()
            self.last_seq = msg.get("seq")
            return None
        if t == "ping":
            return {"type": "pong", "t": msg.get("t")}
        return {"type": "error", "error": f"unknown type: {t}"}

    # ---- 제어 한 스텝 ----
    def tick(self):
        now = time.monotonic()
        self.watchdog = (now - self.last_cmd_t) > C.WATCHDOG_TIMEOUT
        sp, st = (0.0, 0.0) if self.watchdog else (self.cmd_speed, self.cmd_steer)
        sp, st, events = self.safety.step(now, sp, st)
        self.out_speed, self.out_steer = sp, st
        self.car.set_speed(sp)
        self.car.set_steer(st)
        return events

    def status(self):
        state = self.safety.state
        if state != "avoiding" and self.watchdog:
            state = "idle"
        return {
            "type": "status",
            "state": state,                 # idle | normal | slow | avoiding
            "avoid_phase": self.safety.phase,
            "dist_cm": None if self.safety.dist is None else round(self.safety.dist, 1),
            "speed": round(self.out_speed, 1),
            "steer": round(self.out_steer, 1),
            "last_seq": self.last_seq,
            "battery_v": self.battery,
        }


async def send(ws, obj):
    if ws is None:
        return
    try:
        await ws.send(json.dumps(obj))
    except ConnectionClosed:
        pass


async def control_loop(ctl):
    period = 1.0 / C.CONTROL_HZ
    status_every = max(1, round(C.CONTROL_HZ / C.STATUS_HZ))
    battery_every = C.CONTROL_HZ * 2
    n = 0
    while True:
        t0 = time.monotonic()
        for ev in ctl.tick():
            print("[event]", ev)
            await send(ctl.client, ev)
        if n % battery_every == 0:
            ctl.battery = ctl.car.battery_v()
        if n % status_every == 0:
            await send(ctl.client, ctl.status())
        n += 1
        await asyncio.sleep(max(0.0, period - (time.monotonic() - t0)))


def make_handler(ctl):
    async def handler(ws):
        old = ctl.client
        ctl.client = ws
        print(f"[conn] client connected: {ws.remote_address}")
        if old is not None:
            await old.close(code=4000, reason="replaced by new client")
        try:
            async for raw in ws:
                try:
                    msg = json.loads(raw)
                    reply = ctl.handle(msg)
                except (ValueError, TypeError) as e:
                    reply = {"type": "error", "error": f"bad message: {e}"}
                if reply:
                    await send(ws, reply)
        except ConnectionClosed:
            pass
        finally:
            if ctl.client is ws:
                ctl.client = None
                ctl.cmd_speed = ctl.cmd_steer = 0.0
                print("[conn] client disconnected, stopping")
    return handler


async def main(args):
    car = FakeCar(args.fake_dist, args.verbose) if args.dry_run else Car()
    ctl = Controller(car, args.max_speed)
    stop = asyncio.Event()
    loop = asyncio.get_running_loop()
    for sig in (signal.SIGINT, signal.SIGTERM):
        loop.add_signal_handler(sig, stop.set)

    task = asyncio.create_task(control_loop(ctl))
    try:
        async with serve(make_handler(ctl), C.HOST, args.port, ping_interval=2, ping_timeout=2):
            print(f"[d455_drive] listening on ws://{C.HOST}:{args.port}  dry_run={args.dry_run}")
            await stop.wait()
    finally:
        task.cancel()
        car.set_speed(0)
        car.close()
        print("[d455_drive] stopped")


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("--port", type=int, default=C.PORT)
    p.add_argument("--max-speed", type=float, default=C.MAX_SPEED)
    p.add_argument("--dry-run", action="store_true", help="하드웨어 없이 실행")
    p.add_argument("--fake-dist", type=float, default=150.0, help="dry-run 초음파 거리(cm)")
    p.add_argument("-v", "--verbose", action="store_true")
    asyncio.run(main(p.parse_args()))
