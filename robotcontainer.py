from ntcore import NetworkTableInstance
from commands.defaultdrivecommand import DefaultDriveCommand
from commands.defaultelevcommand import DefaultElevCommand
from constants import GenericConstants
from commands2 import RunCommand
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

        GenericConstants.ALGAE_GROUND_TRIGGER.onTrue(
            RunCommand(lambda: self.elevator.set_position("ALGAE_GROUND_A0")).until(self.elevator.at_position)
        )

        GenericConstants.ALGAE_L2_TRIGGER.onTrue(
            RunCommand(lambda: self.elevator.set_position("ALGAE_LOW_A2")).until(self.elevator.at_position)
        )

        GenericConstants.ALGAE_L3_TRIGGER.onTrue(
            RunCommand(lambda: self.elevator.set_position("ALGAE_HIGH_A3")).until(self.elevator.at_position)
        )