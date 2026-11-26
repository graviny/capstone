#!/usr/bin/env python3
"""노트북에서 속도/조향 명령을 받아 PiCar를 움직이는 WebSocket 서버.

    sudo python3 server.py              # 실차
    python3 server.py --dry-run -v      # 하드웨어 없이 시험

프로토콜은 README.md 참고.
"""
import argparse
import asyncio
import json
import math
import signal
import time

from websockets.asyncio.server import serve
from websockets.exceptions import ConnectionClosed

import config as C
from hardware import Car, FakeCar
from safety import Safety


class Controller:
    def __init__(self, car, max_speed, safety=Safety, robot_id=None):
        self.car = car
        self.robot_id = robot_id
        self.safety = safety(car)
        self.max_speed = max_speed
        self.cmd_speed = 0.0
        self.cmd_steer = 0.0
        self.cmd_wheels = None          # (left, right) %, 사륜 전용 "wheels" 명령
        self.last_cmd_t = 0.0
        self.last_seq = None
        self.client = None
        self.out_speed = 0.0
        self.out_steer = 0.0
        self.watchdog = True
        self.battery = None

    # ---- 수신 ----
    def handle(self, msg):
        if not isinstance(msg, dict):
            raise ValueError("Expected a JSON object")
        t = msg.get("type")
        if t == "cmd":
            sp = float(msg.get("speed", 0))
            st = float(msg.get("steer", 0))
            if not math.isfinite(sp) or not math.isfinite(st):
                raise ValueError("Speed and steer must be finite")
            self.cmd_speed = max(-self.max_speed, min(self.max_speed, sp))
            self.cmd_steer = max(-C.STEER_MAX_DEG, min(C.STEER_MAX_DEG, st))
            self.cmd_wheels = None
            self.last_cmd_t = time.monotonic()
            self.last_seq = msg.get("seq")
            return None
        if t == "wheels":
            if not hasattr(self.car, "drive_wheels"):
                return {"type": "error", "error": "wheels requires --four-wheel"}
            left, right = float(msg.get("left", 0)), float(msg.get("right", 0))
            if not math.isfinite(left) or not math.isfinite(right):
                raise ValueError("Wheel outputs must be finite")
            clamp = lambda v: max(-self.max_speed, min(self.max_speed, v))
            self.cmd_wheels = (clamp(left), clamp(right))
            self.cmd_speed = self.cmd_steer = 0.0
            self.last_cmd_t = time.monotonic()
            self.last_seq = msg.get("seq")
            return None
        if t == "stop":
            self.cmd_wheels = None
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
        if self.cmd_wheels is not None:
            left, right = (0.0, 0.0) if self.watchdog else self.cmd_wheels
            forward = (left + right) / 2
            sp, _, events = self.safety.step(now, forward, 0.0)
            if sp != forward:           # 전방 차단: 전진 성분이 있으면 정지 (제자리 회전·후진은 허용)
                left = right = 0.0
            self.out_speed, self.out_steer = (left + right) / 2, (right - left) / 2
            self.car.drive_wheels(left, right)
            return events
        sp, st = (0.0, 0.0) if self.watchdog else (self.cmd_speed, self.cmd_steer)
        sp, st, events = self.safety.step(now, sp, st)
        self.out_speed, self.out_steer = sp, st
        self.car.drive(sp, st)
        return events

    def status(self):
        state = self.safety.state
        if state != "avoiding" and self.watchdog:
            state = "idle"
        status = {
            "type": "status",
            "robot_id": self.robot_id,      # 여러 대를 구분하기 위한 이름 (--robot-id)
            "drive": getattr(self.car, "drive_type", "ackermann"),  # ackermann | differential
            "max_speed": self.max_speed,
            "state": state,                 # idle | normal | slow | avoiding | blocked(사륜)
            "avoid_phase": self.safety.phase,
            "dist_cm": None if self.safety.dist is None else round(self.safety.dist, 1),
            "speed": round(self.out_speed, 1),
            "steer": round(self.out_steer, 1),
            "last_seq": self.last_seq,
            "battery_v": self.battery,
        }
        if hasattr(self.car, "command"):
            status.update(left=round(self.car.command.left * 100, 1),
                          right=round(self.car.command.right * 100, 1))
        return status


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
        ctl.cmd_speed = ctl.cmd_steer = 0.
        ctl.cmd_wheels = None
        ctl.last_cmd_t = 0.
        print(f"[conn] client connected: {ws.remote_address}")
        if old is not None:
            await old.close(code=4000, reason="replaced by new client")
        try:
            async for raw in ws:
                try:
                    msg = json.loads(raw)
                    if ctl.client is not ws:
                        break
                    reply = ctl.handle(msg)
                except (ValueError, TypeError, OverflowError) as e:
                    reply = {"type": "error", "error": f"bad message: {e}"}
                if reply:
                    await send(ws, reply)
        except ConnectionClosed:
            pass
        finally:
            if ctl.client is ws:
                ctl.client = None
                ctl.cmd_wheels = None
                ctl.cmd_speed = ctl.cmd_steer = 0.0
                print("[conn] client disconnected, stopping")
    return handler


async def run_server(args, car, safety):
    ctl = Controller(car, args.max_speed, safety, args.robot_id)
    stop = asyncio.Event()
    loop = asyncio.get_running_loop()
    for sig in (signal.SIGINT, signal.SIGTERM):
        loop.add_signal_handler(sig, stop.set)

    task = asyncio.create_task(control_loop(ctl))
    stop_task = asyncio.create_task(stop.wait())
    try:
        async with serve(make_handler(ctl), args.host, args.port, ping_interval=2, ping_timeout=2):
            print(f"[d455_drive] robot={args.robot_id} listening on ws://{args.host}:{args.port}"
                  f"  max_speed={args.max_speed} dry_run={args.dry_run}")
            done, _ = await asyncio.wait((stop_task, task), return_when=asyncio.FIRST_COMPLETED)
            if task in done:
                task.result()
    finally:
        task.cancel()
        stop_task.cancel()
        try:
            car.drive(0, 0)
        finally:
            car.close()
            await asyncio.gather(task, stop_task, return_exceptions=True)
        print("[d455_drive] stopped")


async def main(args):
    if args.four_wheel:
        from four_wheel import FourWheelCar, FrontStop, MAX_SPEED, hardware_context
        args.host = args.host or C.HOST
        args.robot_id = args.robot_id or "picar4wd"
        args.max_speed = MAX_SPEED if args.max_speed is None else args.max_speed
        if not 0 < args.max_speed <= MAX_SPEED:
            raise SystemExit("Four-wheel --max-speed must be within (0, 30]")
        if not args.dry_run and not args.vendor_dir:
            raise SystemExit("Four-wheel hardware requires --vendor-dir")
        with hardware_context(args.dry_run, args.vendor_dir, args.max_speed, args.servo_angle) as hardware:
            await run_server(args, FourWheelCar(hardware, args.fake_dist, args.allow_no_echo), FrontStop)
        return

    args.host = args.host or C.HOST
    args.robot_id = args.robot_id or "picar_b2"
    args.max_speed = C.MAX_SPEED if args.max_speed is None else args.max_speed
    car = FakeCar(args.fake_dist, args.verbose) if args.dry_run else Car()
    await run_server(args, car, Safety)


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("--port", type=int, default=C.PORT)
    p.add_argument("--host", help=f"기본 {C.HOST} (노트북이 Tailscale/Wi-Fi로 접속)")
    p.add_argument("--robot-id", help="status에 실리는 로봇 이름 (기본 picar_b2 / 사륜 picar4wd)")
    p.add_argument("--allow-no-echo", action="store_true",
                   help="사륜: 초음파 무반향(2m 포화)을 '앞이 비어 있음'으로 간주")
    p.add_argument("--max-speed", type=float)
    p.add_argument("--four-wheel", action="store_true", help="Use the Adeept OrdinaryWheels four-motor chassis")
    p.add_argument("--vendor-dir", help="OrdinaryWheels directory containing Move.py")
    p.add_argument("--servo-angle", type=float, default=90., help="Front sensor center in four-wheel mode")
    p.add_argument("--dry-run", action="store_true", help="하드웨어 없이 실행")
    p.add_argument("--fake-dist", type=float, default=150.0, help="dry-run 초음파 거리(cm)")
    p.add_argument("-v", "--verbose", action="store_true")
    asyncio.run(main(p.parse_args()))
