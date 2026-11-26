"""Run from PowerShell: python demo.py [menu|status|forward|backward|rotate|stop]."""

import argparse
import getpass
import math
import sys
import time

from robot_driver import XiaomiRobot, validate_motion


def status_after_manual_exit(robot, *, attempts=6, interval=0.5):
    """Allow delayed state updates without issuing more control commands."""
    for attempt in range(attempts):
        status = robot.status()
        if status.state_code not in (4, 7):
            return status
        if attempt + 1 < attempts:
            time.sleep(interval)
    return status


def show_exit_status(robot):
    status = status_after_manual_exit(robot)
    print("수동 모드 종료 요청 후 상태:")
    print(status)
    if status.state_code in (4, 7):
        print("종료 응답은 받았지만 상태에서는 수동 모드 종료가 확인되지 않았습니다.")
        print("본체의 시작/일시정지 버튼으로 멈춘 뒤 상태를 다시 확인하세요.")


def parameters(args, command):
    if not math.isfinite(args.seconds) or not 0.1 <= args.seconds <= 5.0:
        raise ValueError("--seconds는 0.1~5.0 사이여야 합니다. 2초 초과는 0.1m/s 이하 직진에서 가능합니다.")
    if not math.isfinite(args.speed) or not 0 < args.speed <= 0.15:
        raise ValueError("--speed는 0보다 크고 0.15 이하여야 합니다.")
    motion = {
        "rotation": args.rotation if command == "rotate" else 0,
        "velocity": 0.0 if command == "rotate" else args.speed,
        "duration_ms": round(args.seconds * 1000),
    }
    if command == "backward":
        motion["velocity"] *= -1
    validate_motion(**motion)
    return motion


def execute(robot, args, command):
    if command == "status":
        print(robot.status())
    elif command == "stop":
        robot.stop()
        print("수동 모드 종료 요청에 응답했습니다.")
        show_exit_status(robot)
    else:
        motion = parameters(args, command)
        print(
            "주행 요청: {velocity}m/s, {rotation}도/초, {duration_ms}ms".format(**motion)
        )
        status = robot.status()
        print("실행 전: 상태={}, 배터리={}%, 오류={} ({})".format(
            status.state, status.battery, status.error, status.error_code
        ))
        if status.error_code:
            raise RuntimeError("청소기 오류를 먼저 확인하세요: " + status.error)
        print("수동 모드 진입을 확인한 뒤 5초 대기하고 이동 명령을 보냅니다.")
        print("Ctrl+C로 종료 요청 가능. 명령이 끝날 때까지 기다리세요.")
        result = robot.move_for(**motion)
        print("수동 모드 종료 응답:", result)
        print("주행 요청 처리가 끝났습니다. 실제 이동 거리와 각도는 확인하세요.")
        show_exit_status(robot)


def main(argv=None):
    parser = argparse.ArgumentParser(description="샤오미 1세대 WES 독립 주행 데모")
    parser.add_argument("command", nargs="?", default="menu",
                        choices=["menu", "status", "forward", "backward", "rotate", "stop"])
    parser.add_argument("--ip", default="192.168.0.12")
    parser.add_argument("--speed", type=float, default=0.1, help="전후진 속도(m/s), 최대 0.15")
    parser.add_argument("--rotation", type=int, default=30, help="회전 속도(도/초), -60~60")
    parser.add_argument("--seconds", type=float, default=1.0,
                        help="주행 시간(초), 0.1~2.0. 0.1m/s 이하 직진은 최대 5초")
    parser.add_argument("--dry-run", action="store_true", help="기기에 접속하지 않고 명령 확인")
    args = parser.parse_args(argv)
    try:
        if args.command in ("forward", "backward", "rotate"):
            motion = parameters(args, args.command)
        else:
            motion = None
        if args.dry_run:
            print("DRY RUN: 기기에 접속하지 않았습니다.")
            print({"ip": args.ip, "command": args.command, "motion": motion})
            return 0

        print("대상 IP:", args.ip)
        print("움직임을 시험할 때는 앞뒤가 비어 있는 평평한 바닥에 두세요.")
        token = getpass.getpass("청소기 32자리 토큰 (화면에 표시되지 않음): ").strip()
        robot = XiaomiRobot.connect(args.ip, token)
        if args.command != "menu":
            execute(robot, args, args.command)
            return 0

        print("\n1 상태 조회 / 2 전진 / 3 후진 / 4 회전 / 5 수동 모드 종료 / 0 프로그램 종료")
        commands = {"1": "status", "2": "forward", "3": "backward", "4": "rotate", "5": "stop"}
        while True:
            choice = input("선택: ").strip()
            if choice == "0":
                return 0
            command = commands.get(choice)
            if command is None:
                print("0~5 중 하나를 입력하세요.")
                continue
            try:
                execute(robot, args, command)
            except Exception as error:
                print("실패:", error, file=sys.stderr)
                print("통신이 안 되면 본체 버튼으로 정지하세요.", file=sys.stderr)
    except KeyboardInterrupt:
        # move_for handles cleanup when interrupted during a motion request.
        print("\n중단했습니다. 연결이 끊겼다면 본체 버튼으로 정지하세요.")
        return 130
    except ModuleNotFoundError as error:
        if error.name == "miio":
            print("현재 Python에 라이브러리가 없습니다. 아래 명령을 실행하세요:", file=sys.stderr)
            print("python -m pip install -r requirements.txt", file=sys.stderr)
        else:
            print("필요한 패키지를 찾지 못했습니다:", error, file=sys.stderr)
        return 1
    except Exception as error:
        print("실패:", error, file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
