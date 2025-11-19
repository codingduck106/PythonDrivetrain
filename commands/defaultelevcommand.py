from commands2 import Command
from subsystems.elevator import Elevator
from typing import Callable

class DefaultElevCommand(Command):
    def __init__(self, elevatorSubsystem: Elevator, movement: Callable[[], float]) -> None:
        self.elevator = elevatorSubsystem
        self.movement = movement
        self.addRequirements(self.elevator)

    def execute(self) -> None:
        if (self.movement() > 0.1):
            self.elevator.set_voltage(self.movement() * 12)
        else:
            self.elevator.set_voltage(0.0)