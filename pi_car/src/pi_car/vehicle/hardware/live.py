"""Explicit opt-in Adeept bootstrap. Imported hardware code is never run in dry-run."""
import importlib.util
import math
from pathlib import Path
import time
from pi_car.vehicle.hardware.adeept import AdeeptMotor
from pi_car.vehicle.perception import RangeReading


class FreshUltrasonic:
    """Retain each raw GPIO Zero echo's time, never re-stamp a cached distance.

    GPIO Zero 2.x samples _read() in its own thread. A single clipped echo
    leaves the last measured distance and timestamp intact. Missing echoes and
    repeated clipping invalidate it. The runtime still expires old readings.
    """
    def __init__(self, *, echo=24, trigger=23, sensor_class=None):
        if sensor_class is None:
            from gpiozero import DistanceSensor
            sensor_class=DistanceSensor
        self.latest=None
        self.previous_saturated = False
        self.last_saturated_at = None
        owner=self
        class TimestampedSensor(sensor_class):
            def _read(self):
                try:
                    value=super()._read()
                    owner.sample(value)
                    return value
                except Exception:
                    owner.latest=None
                    raise
        self.sensor=TimestampedSensor(echo=echo,trigger=trigger,max_distance=2.,queue_len=1,partial=True)

    def sample(self, normalized):
        saturated = normalized == 1.
        isolated_saturation = saturated and not self.previous_saturated
        self.previous_saturated = saturated
        self.last_saturated_at = time.monotonic() if saturated else None
        if isolated_saturation:
            return
        if (normalized is None or not math.isfinite(normalized) or not 0 < normalized < 1):
            self.latest=None
        else:
            self.latest=RangeReading(float(normalized)*2.,0.,time.monotonic())

    def read(self):
        return self.latest

    def read_allow_clear(self):
        reading=self.read()
        if reading is not None:
            return reading
        if self.previous_saturated and self.last_saturated_at is not None:
            return RangeReading(2.,0.,self.last_saturated_at)
        return None

    def close(self):
        self.sensor.close()
        self.latest=None


class AdeeptHardware:
    """One PCA9685 owner; centers front servo before any driving is possible."""
    def __init__(self,vendor_dir,*,max_output=.2,servo_angle=90.,echo=24,trigger=23):
        if not math.isfinite(max_output) or not 0 < max_output <= .3:
            raise ValueError('Initial hardware max-output must be within (0, 0.3]')
        if not math.isfinite(servo_angle) or not 0 <= servo_angle <= 180:
            raise ValueError('Servo center angle must be 0..180 degrees')
        self.path=Path(vendor_dir).expanduser().resolve()/'Move.py'
        if not self.path.is_file():raise ValueError('vendor-dir must contain the installed Adeept Move.py')
        self.max_output,self.servo_angle=max_output,servo_angle
        self.echo,self.trigger=echo,trigger
        self.move=self.sensor=self.servo=None

    def __enter__(self):
        spec=importlib.util.spec_from_file_location('_pi_car_adeept_move',self.path)
        self.move=importlib.util.module_from_spec(spec)
        spec.loader.exec_module(self.move)
        for name in ('setup','motorStop'):
            if not hasattr(self.move,name):raise ValueError(f'Incompatible Move.py: missing {name}')
        try:
            self.move.setup()
            self.move.motorStop()
            self.motor=AdeeptMotor(self.move,enabled=True,max_output=self.max_output)
            from adafruit_motor.servo import Servo
            # Use the motor module's shared PCA9685 instead of importing Functions
            # or starting a competing vendor I2C/automatic-driving thread.
            self.servo=Servo(self.move.pwm_motor.channels[0],min_pulse=500,max_pulse=2400,actuation_range=180)
            self.servo.angle=self.servo_angle
            time.sleep(.5)
            self.sensor=FreshUltrasonic(echo=self.echo,trigger=self.trigger)
            return self
        except BaseException:
            self.close()
            raise

    def close(self):
        try:
            if self.move and hasattr(self.move,'motor1'):
                self.move.motorStop()
        finally:
            try:
                if self.sensor:self.sensor.close()
            finally:
                try:
                    if self.servo:self.servo.angle=None
                finally:
                    if self.move and hasattr(self.move,'pwm_motor'):
                        self.move.pwm_motor.deinit()

    def __exit__(self,*_):
        self.close()
