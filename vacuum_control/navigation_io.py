"""Camera acquisition and serialized miIO control, with independent watchdogs."""
from datetime import datetime
import json
import logging
import math
from pathlib import Path
import socket
import threading
import time
from uuid import uuid4

import cv2
import numpy as np

from replan_measurement import append_measurement

ROOT = Path(__file__).resolve().parent
LOGGER = logging.getLogger(__name__)


def is_timeout(error):
    return isinstance(error.__cause__ or error, (TimeoutError, socket.timeout))


class CameraWorker:
    def __init__(self, width=640, height=480):
        self.width,self.height = width,height
        self.lock = threading.Lock()
        self.packet = None
        self.error = "D455 연결 중"
        self.closing = threading.Event()
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def _run(self):
        capture = None
        try:
            from realsense_camera import RealSenseCapture
            capture = RealSenseCapture(width=self.width,height=self.height)
            metadata = capture.metadata()
            intrinsics = capture._profile.get_stream(capture.rs.stream.color).as_video_stream_profile().get_intrinsics()
            def ray(pixel):
                return np.asarray(capture.rs.rs2_deproject_pixel_to_point(
                    intrinsics, [float(pixel[0]), float(pixel[1])], 1.0), float)
            previous = None
            missed_frame = False
            while not self.closing.is_set():
                try:
                    ok, frame = capture.read()
                except RuntimeError as error:
                    if "Frame didn't arrive within" not in str(error):
                        raise
                    ok = False
                if not ok:
                    if missed_frame:
                        raise RuntimeError("D455 프레임이 연속으로 들어오지 않습니다. 카메라 연결을 확인하세요.")
                    missed_frame = True
                    with self.lock:
                        self.error = "D455 프레임 대기 중 · 다시 수신 시도"
                    self.closing.wait(0.05)
                    continue
                number = capture.frame_number
                if previous == number:
                    # Alignment may reuse a colour frame while depth catches
                    # up. Keep waiting without refreshing the observation age;
                    # the GUI/motion watchdog still stops on genuinely stale
                    # input, and a subsequent new frame can recover normally.
                    self.closing.wait(0.01)
                    continue
                previous = number
                missed_frame = False
                packet = {"frame": frame, "depth": capture._depth_image,
                          "scale": capture.depth_scale, "stamp": time.monotonic(),
                          "received_perf": time.perf_counter(),
                          "number": number, "metadata": metadata, "ray": ray}
                with self.lock:
                    self.packet = packet
                    self.error = ""
        except Exception as error:
            with self.lock:
                self.packet = None
                self.error = str(error)
        finally:
            if capture:
                capture.release()

    def latest(self):
        with self.lock:
            return self.packet, self.error

    def close(self):
        self.closing.set()
        self.thread.join(timeout=0.5)


class NativeMiio:
    """Pinned python-miio 0.5.12 with a short socket timeout and no protocol retries."""
    def __init__(self, ip, token, turn_sign=1, sequence_file=None):
        import ipaddress
        ipaddress.IPv4Address(ip)
        if len(token) != 32 or any(c not in "0123456789abcdefABCDEF" for c in token):
            raise ValueError("기기 토큰은 32자리 16진수여야 합니다")
        if turn_sign not in (-1, 1):
            raise ValueError("turn_sign은 -1 또는 1입니다")
        from miio import RoborockVacuum
        from miio.miioprotocol import MiIOProtocol
        from miio.exceptions import DeviceException
        class BoundedProtocol(MiIOProtocol):
            def send_handshake(self, *, retry_count=0):
                # 0.5.12's standard handshake uses an independent 5s timeout.
                message = MiIOProtocol.discover(self.ip,timeout=0.4)
                if message is None:
                    raise DeviceException("청소기 연결 시간 초과") from TimeoutError("miIO 핸드셰이크 응답 없음")
                self._device_id = message.header.value.device_id
                self._device_ts = message.header.value.ts
                self._discovered = True
                return message
        class ShortTimeoutVacuum(RoborockVacuum):
            timeout = 0.4
            retry_count = 0
        self.path = Path(sequence_file or ROOT/"runtime"/("miio_"+ip.replace(".","_")+".json"))
        self.sequence_temp = self.path.with_suffix(f".{uuid4().hex}.tmp")
        self.sequence_save_due = 0.0
        seeds = []
        paths = [self.path]
        try:
            from appdirs import user_cache_dir
            paths.append(Path(user_cache_dir("python-miio"))/"python-mirobo.seq")
        except ImportError:
            pass
        for path in paths:
            try:
                seeds.append(int(json.loads(path.read_text(encoding="utf-8"))["seq"]))
            except (OSError, ValueError, KeyError, TypeError):
                pass
        seed = (max(seeds, default=1000)+23) % 9900
        self.device = ShortTimeoutVacuum(ip, token, start_id=seed, model="rockrobo.vacuum.v1")
        self.device._protocol = BoundedProtocol(ip,token,start_id=seed,timeout=0.4)
        self.turn_sign = turn_sign

    def _save_sequence(self):
        try:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            self.sequence_temp.write_text(json.dumps({"seq": self.device._protocol.raw_id}),encoding="utf-8")
            self.sequence_temp.replace(self.path)
        except OSError as error:
            # The sequence file is a restart aid, separate from the robot reply.
            LOGGER.warning("miIO 통신 번호 저장 실패: %s",error)
            try:
                self.sequence_temp.unlink(missing_ok=True)
            except OSError:
                pass

    def _call(self, function, *args, periodic_save=False, **kwargs):
        try:
            return function(*args, **kwargs)
        finally:
            now = time.monotonic()
            if not periodic_save or now >= self.sequence_save_due:
                self.sequence_save_due = now+1.0
                self._save_sequence()

    def _status(self, *, retry_on_timeout=False):
        try:
            return self._call(self.device.status)
        except Exception as error:
            if not retry_on_timeout or not is_timeout(error):
                raise
        return self._call(self.device.status)

    def check(self, *, retry_on_timeout=False):
        status = self._status(retry_on_timeout=retry_on_timeout)
        if status.error_code:
            raise RuntimeError("청소기 오류: " + status.error)
        if status.battery < 15:
            raise RuntimeError("배터리가 15% 미만입니다. 충전 후 시험하세요")
        if status.state_code not in (2, 3, 4, 7, 10):
            raise RuntimeError("청소/충전/복귀를 멈추고 바닥에 놓으세요: " + status.state)
        return status

    def start(self):
        self._call(self.device.manual_start)
        for _ in range(8):
            if self._status(retry_on_timeout=True).state_code == 7:
                return
            time.sleep(0.15)
        raise RuntimeError("수동 모드 진입을 확인하지 못했습니다")

    def move(self, velocity, rotation):
        if not math.isfinite(velocity) or not -0.1 <= velocity <= 0.1 or abs(rotation) > 30:
            raise ValueError("제어값이 데모 속도 제한을 넘었습니다")
        self._call(self.device.manual_control, rotation=int(rotation*self.turn_sign),
                   velocity=float(velocity), duration=500, periodic_save=True)

    def confirm_manual_mode(self):
        if self.check().state_code != 7:
            raise RuntimeError("청소기가 수동 모드를 벗어났습니다")

    def _require_stopped_status(self):
        status = self.device.status()
        if status.state_code == 7 and not status.error_code:
            # The firmware can briefly report manual mode after acknowledging stop.
            time.sleep(0.30)
            status = self.device.status()
        if status.state_code not in (3, 10) or status.error_code:
            raise RuntimeError(f"청소기 정지 상태 미확인: {status.state} (오류 {status.error_code})")
        return status.state

    def _timed_stop_command(self, stop_started_at, name, function, kwargs):
        call_started_at = datetime.now().astimezone().isoformat(timespec="milliseconds")
        started = time.perf_counter()
        result, detail = "responded", ""
        try:
            reply = self._call(function, **kwargs)
            if name == "get_status_after_stop":
                detail = reply
        except Exception as error:
            cause = error.__cause__ or error
            result = "timeout" if is_timeout(error) else "error"
            detail = f"{type(cause).__name__}: {cause}"
        return {"stop_started_at": stop_started_at, "call_started_at": call_started_at,
                "command": name, "result": result,
                "call_ms": round((time.perf_counter()-started)*1000, 3), "detail": detail}

    def halt(self):
        commands = (
            ("app_rc_move_zero", self.device.manual_control, {"rotation": 0, "velocity": 0.0, "duration": 300}),
            ("app_rc_end", self.device.manual_stop, {}),
            ("app_pause", self.device.pause, {}),
        )
        stop_started_at = datetime.now().astimezone().isoformat(timespec="milliseconds")
        timings = [self._timed_stop_command(stop_started_at, *command) for command in commands]
        failures = [row for row in timings if row["result"] != "responded"]
        # Attempt one more stop-only command after the initial pause has already
        # been sent. Keep all the original attempts in the log.
        for command, row in zip(reversed(commands[1:]), reversed(timings[1:])):
            if row["result"] == "timeout":
                timings.append(self._timed_stop_command(stop_started_at, *command))
                break

        replied = {row["command"] for row in timings if row["result"] == "responded"}
        if (failures and all(row["result"] == "timeout" for row in failures)
                and {"app_rc_end", "app_pause"} <= replied):
            time.sleep(0.15)
            status = self._timed_stop_command(stop_started_at, "get_status_after_stop",
                                              self._require_stopped_status, {})
            timings.append(status)
            if status["result"] == "responded":
                failures.clear()
            else:
                failures.append(status)

        # Keep diagnostic writes out of the interval between stop commands.
        try:
            for row in timings:
                append_measurement(self.path.with_name("stop_command_timing.csv"), row)
        except OSError as error:
            LOGGER.warning("정지 기록 저장 실패: %s", error)
        if failures:
            names = ", ".join(row["command"] for row in failures)
            raise RuntimeError(f"정지 상태 미확인 ({names}). 본체 버튼으로 정지하세요")


class MotionWorker:
    """One I/O thread; no stale command queue, and no implicit rearm on faults."""
    MOVE_INTERVAL = 0.30

    def __init__(self, factory, clock=time.monotonic, *, event_log=None):
        self.factory, self.clock = factory, clock
        self.event_log = event_log
        self.lock = threading.Lock()
        self.request = (False, 0.0, 0, 0.0)
        self.stop_generation = 0
        self.stop_reason = "주행 해제"
        self.confirmed_generation = -1
        self.status = "실물 제어 대기"
        self.fault = ""
        self.client = None
        self.active = False
        self.next_move_at = 0
        self.missed_move_reply = False
        self.verify_link = False
        self.closing = threading.Event()
        self.thread = None

    def start_thread(self):
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def submit(self, velocity=0.0, rotation=0, stamp=0.0, enabled=False, *, stop_reason="주행 해제"):
        with self.lock:
            if not enabled and self.request[0]:
                self.stop_generation += 1
                self.stop_reason = stop_reason
            self.request = bool(enabled), float(velocity), int(rotation), float(stamp)

    def stop_confirmed(self):
        with self.lock:
            return not self.fault and not self.request[0] and self.confirmed_generation==self.stop_generation

    def _record_event(self, event, detail, *, stamp=None):
        if self.event_log is None:
            return
        try:
            append_measurement(self.event_log, {
                "time": stamp or datetime.now().astimezone().isoformat(timespec="milliseconds"),
                "event": event, "detail": detail,
            })
        except OSError as error:
            LOGGER.warning("주행 기록 저장 실패: %s",error)

    def _stop_active(self, reason="프로그램 종료"):
        if self.active:
            stamp = datetime.now().astimezone().isoformat(timespec="milliseconds")
            try:
                self.client.halt()
            finally:
                self.active = False
                self.missed_move_reply = self.verify_link = False
                self._record_event("stop",reason,stamp=stamp)

    def tick(self):
        with self.lock:
            enabled, velocity, rotation, stamp = self.request
            generation = self.stop_generation
            pending_stop = generation > max(0,self.confirmed_generation)
            stop_reason = self.stop_reason
        if self.fault:
            return
        phase = "정지"
        try:
            age = self.clock()-stamp
            stale = not 0 <= age <= 0.35
            if pending_stop or not enabled or stale:
                reason = stop_reason if pending_stop or not enabled else f"D455 측정 갱신 지연 ({age*1000:.0f}ms)"
                self._stop_active(reason)
                if pending_stop or not enabled:
                    with self.lock:
                        if self.stop_generation==generation:
                            self.confirmed_generation = generation
                self.status = "정지 / 측정값 대기"
                return
            if not self.active:
                phase = "주행 전 청소기 상태 조회"
                if self.client is None:
                    self.client = self.factory()
                self.client.check(retry_on_timeout=True)
                # A request can time out after reaching the robot, so cleanup is
                # needed even if the start RPC itself fails.
                self.active = True
                phase = "수동 모드 진입 확인"
                self.client.start()
                self.next_move_at = self.clock()+5.0
                self.status = "수동 모드 준비 중 (약 5초)"
                return
            if self.clock() < self.next_move_at:
                return
            # Recheck freshness after potentially slow I/O during initialization.
            with self.lock:
                enabled, velocity, rotation, stamp = self.request
                pending_stop = self.stop_generation != generation
                stop_reason = self.stop_reason
            now = self.clock()
            age = now-stamp
            if pending_stop or not enabled or not 0 <= age <= 0.35:
                reason = stop_reason if pending_stop or not enabled else f"D455 측정 갱신 지연 ({age*1000:.0f}ms)"
                self._stop_active(reason)
                return
            if self.verify_link:
                phase = "이동 응답 누락 후 청소기 상태 조회"
                self.client.confirm_manual_mode()
                self.verify_link = False
                self.status = "연결 확인 · 최신 경로 대기"
                self._record_event("link_recovered",self.status)
                return
            phase = "이동 명령(app_rc_move)"
            self.next_move_at = now+self.MOVE_INTERVAL
            try:
                self.client.move(velocity, rotation)
            except Exception as error:
                if self.missed_move_reply or not is_timeout(error):
                    raise
                self.missed_move_reply = self.verify_link = True
                self.status = "이동 응답 누락 · 연결 확인 중"
                LOGGER.warning(self.status)
                self._record_event("move_reply_timeout",str(error))
                return
            self.missed_move_reply = False
            self.status = "실물 제어 중"
        except Exception as error:
            self.fault = f"{phase}: {error}"
            self.status = "오류 — 재시작 필요"
            try:
                self._stop_active(self.fault)
            except Exception as stop_error:
                self.fault += f" / {stop_error}"
            self._record_event("fault",self.fault)

    def _run(self):
        try:
            while not self.closing.is_set():
                self.tick()
                self.closing.wait(0.05)
        finally:
            try:
                self._stop_active()
            except Exception:
                self.fault = "종료 중 정지 통신 실패: 본체 버튼 사용"
                self._record_event("fault",self.fault)

    def close(self):
        self.submit(stop_reason="프로그램 종료")
        self.closing.set()
        if self.thread:
            self.thread.join(timeout=3.0)
        else:
            self._stop_active()
