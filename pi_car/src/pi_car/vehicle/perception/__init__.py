from dataclasses import dataclass
import math


@dataclass(frozen=True)
class RangeReading:
    distance_m: float
    angle_rad: float  # 0 front, CCW positive in robot frame
    received_monotonic: float

    def usable(self, now, timeout_s=.2):
        return (math.isfinite(self.distance_m) and self.distance_m > 0
                and math.isfinite(self.angle_rad)
                and 0 <= now-self.received_monotonic <= timeout_s)
