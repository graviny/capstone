"""Bounded motion adapter for a first-generation Xiaomi Mi Robot Vacuum.

This adapter executes timed motion. It does not measure position, detect arrival,
read products, or complete WES tasks. No connection is made at import time.
"""

import ipaddress
import math
import time
import warnings


def validate_motion(rotation, velocity, duration_ms):
    """Conservative limits for the first physical demonstration."""
    if not isinstance(rotation, int) or not -60 <= rotation <= 60:
        raise ValueError("회전 속도는 -60~60 사이의 정수(도/초)여야 합니다.")
    if not math.isfinite(velocity) or not -0.15 <= velocity <= 0.15:
        raise ValueError("이 데모의 전진 속도 범위는 -0.15~0.15m/s입니다.")
    max_duration_ms = 5000 if rotation == 0 and 0 < velocity <= 0.1 else 2000
    if not isinstance(duration_ms, int) or not 100 <= duration_ms <= max_duration_ms:
        raise ValueError(f"이 속도·방향에서 주행 시간은 100~{max_duration_ms}ms여야 합니다.")


class XiaomiRobot:
    """Reusable driver; device injection also permits tests without hardware."""

    def __init__(self, device):
        self.device = device

    @classmethod
    def connect(cls, ip, token):
        ipaddress.IPv4Address(ip)
        if len(token) != 32 or any(c not in "0123456789abcdefABCDEF" for c in token):
            raise ValueError("토큰은 0~9, a~f로 구성된 32자리 값이어야 합니다.")
        from miio import RoborockVacuum

        return cls(RoborockVacuum(ip=ip, token=token))

    def status(self):
        return self.device.status()

    def stop(self):
        """Request exit from manual mode; this requires a working connection."""
        return self.device.manual_stop()

    def _move_with_extended_wait(self, rotation, velocity, duration_ms):
        # miio 0.5.12 always stops five seconds after sending a move. Preserve
        # its startup wait and give the longer command one second of margin.
        self.device.manual_start()
        for attempt in range(3):
            if self.status().state_code == 7:
                time.sleep(5)
                self.device.manual_control(rotation=rotation, velocity=velocity, duration=duration_ms)
                time.sleep(duration_ms / 1000 + 1)
                return self.stop()
            if attempt < 2:
                time.sleep(2)

    def move_for(self, *, rotation=0, velocity=0.1, duration_ms=1000):
        """Send one timed manual command and wait before exiting manual mode.

        Units: rotation in degrees/second, velocity in metres/second, duration in
        milliseconds. Successful return confirms command exchanges and elapsed
        time, not the actual distance travelled or target arrival.
        """
        validate_motion(rotation, velocity, duration_ms)
        try:
            if duration_ms > 4000:
                result = self._move_with_extended_wait(rotation, velocity, duration_ms)
            else:
                result = self.device.manual_control_once(
                    rotation=rotation, velocity=velocity, duration=duration_ms
                )
            # In 0.5.12, None means activation retries ran out without a move.
            if result is None:
                raise RuntimeError(
                    "수동 모드 진입을 확인하지 못해 이동 명령이 전송되지 않았습니다."
                )
        except BaseException:
            # A timeout can occur after a command reached the device. Always try
            # to exit manual mode, preserving the original failure if this fails.
            try:
                self.stop()
            except Exception:
                warnings.warn(
                    "수동 모드 종료 요청도 실패했습니다. 본체 버튼으로 정지하세요.",
                    RuntimeWarning,
                )
            raise
        return result
