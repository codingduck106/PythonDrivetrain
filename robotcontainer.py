from ntcore import NetworkTableInstance
from commands.defaultdrivecommand import DefaultDriveCommand
from commands.defaultelevcommand import DefaultElevCommand
from constants import GenericConstants

from subsystems.drive import SwerveDrive
from subsystems.elevator import Elevator

class RobotContainer:

    def __init__(self, ntInstance: NetworkTableInstance):

        self.drive = SwerveDrive(ntInstance)

        self.elevator = Elevator(ntInstance)


        self.configure_bindings()


    def configure_bindings(self):
        self.drive.setDefaultCommand(
            DefaultDriveCommand(
                self.drive,
                GenericConstants.DriverConstants.DRIVER_CONTROLLER
            )
        )
        self.elevator.setDefaultCommand(
            DefaultElevCommand(
                self.elevator,
                lambda: GenericConstants.OperatorConstants.OPERATOR_CONTROLLER.getLeftY()
            )
        )