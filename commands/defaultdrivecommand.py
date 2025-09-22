from subsystems.drivetrain import Drive
from commands2 import Command
from typing import Callable

class DefaultDriveCommand(Command):
    """A custom command to drive the drivetrain."""

    def __init__(self, drive: Drive, xSupplier: Callable[[], float], ySupplier: Callable[[], float], rotSupplier: Callable[[], float]) -> None:
        """Initializes the command.
        
        :param drive: the drivetrain instance
        :param xSupplier: a function that returns a float. We use it because of lambda functions outputting joystick values. Check robotcontainer.py!
        :param ySupplier: same as above
        :param rotSupplier: same as above, again."""
        self.drive = drive
        self.x_supplier = xSupplier
        self.y_supplier = ySupplier
        self.rotation_supplier = rotSupplier

        self.addRequirements(self.drive) # adds a subsytem to be required for the command to run.

    def execute(self) -> None:
        """Function that says what happens when you execute the command."""
        self.drive.drive(self.x_supplier(), self.y_supplier(), self.rotation_supplier())