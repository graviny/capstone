from typing import Protocol
from pi_car.common.messages import MotorCommand


class MotorDriver(Protocol):
    def apply(self, command: MotorCommand) -> None: ...
    def stop(self) -> None: ...


class DisabledMotor:
    """Default driver: records commands without importing GPIO or powering motors."""
    def __init__(self):
        self.last_command = MotorCommand()

    def apply(self, command):
        self.last_command = command

    def stop(self):
        self.last_command = MotorCommand()
