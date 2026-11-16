from dataclasses import asdict, dataclass
from enum import Enum
import math


class Mode(str, Enum):
    IDLE = "IDLE"
    NORMAL = "NORMAL"
    OBSTACLE = "OBSTACLE"
    AVOIDANCE = "AVOIDANCE"
    REJOIN = "REJOIN"
    REPLANNING = "REPLANNING"
    EMERGENCY = "EMERGENCY"
    ARRIVED = "ARRIVED"


@dataclass(frozen=True)
class Pose:
    robot_id: str
    x: float
    y: float
    theta: float  # radians, CCW from world +X
    timestamp: float  # host UNIX capture time; PC/Pi clocks must be synchronized
    frame_id: str
    valid: bool = True

    def __post_init__(self):
        if type(self.valid) is not bool:
            raise ValueError("Pose.valid must be boolean")
        if any(type(v) not in (int, float) for v in (self.x, self.y, self.theta, self.timestamp)):
            raise ValueError("Pose values must be numeric")
        if not all(math.isfinite(v) for v in (self.x, self.y, self.theta, self.timestamp)):
            raise ValueError("Pose values must be finite")
        if not self.robot_id or not self.frame_id:
            raise ValueError("robot_id and frame_id are required")

    def to_dict(self):
        return asdict(self)


@dataclass(frozen=True)
class MotorCommand:
    """Normalized left/right output; hardware adapter must calibrate these values."""
    left: float = 0.
    right: float = 0.

    def __post_init__(self):
        if not all(math.isfinite(v) and -1 <= v <= 1 for v in (self.left, self.right)):
            raise ValueError("Motor output must be finite and within [-1, 1]")
