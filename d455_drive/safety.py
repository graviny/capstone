"""초음파 기반 안전 계층.

노트북 명령(speed, steer)을 받아 실제로 내보낼 값을 결정한다.
- normal  : 명령 그대로
- slow    : 앞이 가까우면 전진 속도 감속
- avoiding: 앞이 막히면 노트북 명령을 무시하고 회피 동작 수행
            정지 → 좌/우 스캔 → 후진(빈 쪽으로 머리 돌림) → 빈 쪽으로 전진 선회
회피가 끝나면 이벤트를 내보내고 normal 로 돌아간다. 이후 경로 재계획은 노트북 몫.
"""
from collections import deque

import config as C


class Safety:
    def __init__(self, car):
        self.car = car
        self.state = "normal"
        self.phase = None
        self.phase_t0 = 0.0
        self.scan_left = None
        self.scan_right = None
        self.avoid_dir = None       # "left" / "right" / None(양쪽 막힘)
        self._hist = deque(maxlen=3)
        self.dist = None

    def _filtered(self, raw):
        self._hist.append(raw)
        return sorted(self._hist)[len(self._hist) // 2]

    def _enter(self, phase, now):
        self.phase = phase
        self.phase_t0 = now

    def step(self, now, cmd_speed, cmd_steer):
        """반환: (speed, steer, events[list of dict])"""
        events = []
        self.dist = self._filtered(self.car.distance_cm())

        if self.state == "avoiding":
            return self._avoid_step(now, events)

        if cmd_speed > 0 and self.dist < C.STOP_DIST_CM:
            self.state = "avoiding"
            self.scan_left = self.scan_right = self.avoid_dir = None
            self._enter("stop", now)
            events.append({"type": "event", "event": "avoid_start", "dist_cm": round(self.dist, 1)})
            return 0, 0, events

        if cmd_speed > 0 and self.dist < C.SLOW_DIST_CM:
            self.state = "slow"
            r = (self.dist - C.STOP_DIST_CM) / (C.SLOW_DIST_CM - C.STOP_DIST_CM)
            factor = C.SLOW_MIN_FACTOR + (1 - C.SLOW_MIN_FACTOR) * r
            return cmd_speed * factor, cmd_steer, events

        self.state = "normal"
        return cmd_speed, cmd_steer, events

    def _avoid_step(self, now, events):
        t = now - self.phase_t0
        car = self.car

        if self.phase == "stop":
            if t >= C.AVOID_STOP_TIME:
                car.set_scan(C.SCAN_LEFT_DEG)
                self._enter("scan_left", now)
            return 0, 0, events

        if self.phase == "scan_left":
            if t >= C.SCAN_SETTLE_TIME:
                self.scan_left = car.distance_cm()
                car.set_scan(C.SCAN_RIGHT_DEG)
                self._enter("scan_right", now)
            return 0, 0, events

        if self.phase == "scan_right":
            if t >= C.SCAN_SETTLE_TIME * 1.6:   # 왼쪽 끝 → 오른쪽 끝, 이동 거리가 길다
                self.scan_right = car.distance_cm()
                car.set_scan(0)
                l, r = self.scan_left, self.scan_right
                if max(l, r) < C.CLEAR_DIST_CM:
                    self.avoid_dir = None
                else:
                    self.avoid_dir = "left" if l >= r else "right"
                self._enter("backup", now)
            return 0, 0, events

        side = 1 if self.avoid_dir == "left" else -1

        if self.phase == "backup":
            if t >= C.AVOID_BACKUP_TIME:
                if self.avoid_dir is None:
                    return self._finish(now, events, "blocked")
                self._enter("turn", now)
                return 0, 0, events
            # 후진 시 반대로 조향해야 차 머리가 빈 쪽으로 돈다
            steer = 0 if self.avoid_dir is None else -side * C.STEER_MAX_DEG
            return -C.AVOID_SPEED, steer, events

        if self.phase == "turn":
            # 후진 직후라 정면이 아직 STOP_DIST 안일 수 있음 → 아주 가까울 때만 중단
            if self.dist < C.AVOID_ABORT_CM:
                return self._finish(now, events, "blocked")
            if t >= C.AVOID_TURN_TIME:
                return self._finish(now, events, "ok")
            return C.AVOID_SPEED, side * C.STEER_MAX_DEG, events

        return self._finish(now, events, "blocked")

    def _finish(self, now, events, result):
        events.append({
            "type": "event",
            "event": "avoid_done",
            "result": result,                       # "ok" | "blocked"
            "avoid_dir": self.avoid_dir,
            "scan_left_cm": None if self.scan_left is None else round(self.scan_left, 1),
            "scan_right_cm": None if self.scan_right is None else round(self.scan_right, 1),
        })
        self.state = "normal"
        self.phase = None
        self._hist.clear()
        self.car.set_scan(0)
        return 0, 0, events
