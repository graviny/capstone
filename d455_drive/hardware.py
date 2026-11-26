"""PiCar-B2 하드웨어 래퍼. PCA9685 하나를 모터와 서보가 함께 쓴다."""
import time

import config as C


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


class Car:
    def __init__(self):
        import busio
        from board import SCL, SDA
        from adafruit_pca9685 import PCA9685
        from adafruit_motor import motor, servo
        from gpiozero import DistanceSensor

        i2c = busio.I2C(SCL, SDA)
        self.pca = PCA9685(i2c, address=C.PCA9685_ADDR)
        self.pca.frequency = 50

        self.motor = motor.DCMotor(self.pca.channels[C.MOTOR_IN1], self.pca.channels[C.MOTOR_IN2])
        self.motor.decay_mode = motor.SLOW_DECAY

        def mk_servo(ch):
            return servo.Servo(self.pca.channels[ch], min_pulse=500, max_pulse=2400, actuation_range=180)

        self.steer_servo = mk_servo(C.STEER_CHANNEL)
        self.scan_servo = mk_servo(C.SCAN_CHANNEL)
        self.sensor = DistanceSensor(echo=C.ULTRA_ECHO, trigger=C.ULTRA_TRIG, max_distance=C.ULTRA_MAX_M)

        try:
            import smbus
            self.bus = smbus.SMBus(1)
        except Exception:
            self.bus = None

        self._speed = None
        self._steer = None
        self._scan = None
        self.set_steer(0)
        self.set_scan(0)
        self.set_speed(0)

    # speed: -100..100 (%), 음수 = 후진
    def set_speed(self, speed):
        speed = _clamp(speed, -100, 100)
        if speed == self._speed:
            return
        self._speed = speed
        self.motor.throttle = C.MOTOR_DIRECTION * speed / 100.0

    # steer: 도(deg), + = 왼쪽
    def set_steer(self, deg):
        deg = _clamp(deg, -C.STEER_MAX_DEG, C.STEER_MAX_DEG)
        if self._steer is not None and abs(deg - self._steer) < 0.5:
            return
        self._steer = deg
        self.steer_servo.angle = _clamp(C.SERVO_CENTER + C.STEER_TRIM_DEG + deg, 0, 180)

    def set_scan(self, deg):
        if deg == self._scan:
            return
        self._scan = deg
        self.scan_servo.angle = _clamp(C.SERVO_CENTER + deg, 0, 180)

    def distance_cm(self):
        return self.sensor.distance * 100.0

    def battery_v(self):
        if self.bus is None:
            return None
        try:
            ch = 0
            raw = self.bus.read_byte_data(C.ADC_ADDR, 0x84 | (((ch << 2 | ch >> 1) & 0x07) << 4))
            return round(raw / 255.0 * 4.93 / (1000 / 4000), 2)
        except Exception:
            return None

    def close(self):
        try:
            self.motor.throttle = 0
            self.steer_servo.angle = C.SERVO_CENTER + C.STEER_TRIM_DEG
            self.scan_servo.angle = C.SERVO_CENTER
            time.sleep(0.2)
            self.pca.deinit()
        except Exception:
            pass


class FakeCar:
    """하드웨어 없이 서버/안전 로직을 시험할 때 쓰는 가짜 차량 (--dry-run)."""

    def __init__(self, dist_cm=150.0, verbose=False):
        self.dist = dist_cm
        self.verbose = verbose
        self._speed = 0
        self._steer = 0
        self._scan = 0

    def _log(self, *a):
        if self.verbose:
            print("[fake]", *a)

    def set_speed(self, speed):
        speed = _clamp(speed, -100, 100)
        if speed != self._speed:
            self._log("speed", speed)
        self._speed = speed

    def set_steer(self, deg):
        deg = _clamp(deg, -C.STEER_MAX_DEG, C.STEER_MAX_DEG)
        if deg != self._steer:
            self._log("steer", deg)
        self._steer = deg

    def set_scan(self, deg):
        if deg != self._scan:
            self._log("scan", deg)
        self._scan = deg

    def distance_cm(self):
        return self.dist

    def battery_v(self):
        return None

    def close(self):
        self._log("close")
