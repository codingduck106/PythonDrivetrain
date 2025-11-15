from ntcore import NetworkTableInstance
from wpilib import SmartDashboard
from commands.defaultdrivecommand import DefaultDriveCommand
from constants import GenericConstants
from commands2 import Command
from pathplannerlib.auto import AutoBuilder

from subsystems.drive import SwerveDrive

class RobotContainer:

    def __init__(self, ntInstance: NetworkTableInstance):

        self.drive = SwerveDrive(ntInstance)


        self.configure_bindings()


    def configure_bindings(self):
        self.drive.setDefaultCommand(
            DefaultDriveCommand(
                self.drive,
                GenericConstants.DriverConstants.DRIVER_CONTROLLER
            )
        )
