from subsystems.drivetrain import Drive
from commands2 import Command
import math
from typing import Callable
from constants import *

class DefaultDriveCommand(Command):
    def __init__(self, drive: Drive, xSupplier: Callable[[], float], ySupplier: Callable[[], float], rotSupplier: Callable[[], float]):
        self.drive = drive
        self.x_supplier = xSupplier
        self.y_supplier = ySupplier
        self.rotation_supplier = rotSupplier
        self.addRequirements(self.drive)

    def execute(self):
        self.drive.drive(self.x_supplier(), self.y_supplier(), self.rotation_supplier())