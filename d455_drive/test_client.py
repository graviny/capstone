#!/usr/bin/env python3
"""서버 동작 확인용 간단한 클라이언트.

    python3 test_client.py --speed 30 --steer 0 --time 2
    python3 test_client.py --host 10.107.14.247 --speed 30 --steer 20 --time 1.5

지정 시간 동안 20Hz로 명령을 보내고 받은 상태/이벤트를 출력한 뒤 정지한다.
"""
import argparse
import asyncio
import json
import time

from websockets.asyncio.client import connect


async def main(a):
    async with connect(f"ws://{a.host}:{a.port}") as ws:
        async def reader():
            async for raw in ws:
                print(raw)

        rt = asyncio.create_task(reader())
        t_end = time.monotonic() + a.time
        seq = 0
        while time.monotonic() < t_end:
            await ws.send(json.dumps({"type": "cmd", "speed": a.speed, "steer": a.steer, "seq": seq}))
            seq += 1
            await asyncio.sleep(0.05)
        await ws.send(json.dumps({"type": "stop"}))
        await asyncio.sleep(a.linger)
        rt.cancel()


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("--host", default="127.0.0.1")
    p.add_argument("--port", type=int, default=8765)
    p.add_argument("--speed", type=float, default=0)
    p.add_argument("--steer", type=float, default=0)
    p.add_argument("--time", type=float, default=2.0)
    p.add_argument("--linger", type=float, default=0.5, help="정지 후 상태를 더 받는 시간")
    asyncio.run(main(p.parse_args()))
