from commands2 import Command
from commands2.button import CommandPS4Controller
from subsystems.drive import SwerveDrive
from wpimath.kinematics import ChassisSpeeds
from constants import DriveConstants

class DefaultDriveCommand(Command):
    def __init__(self, drive: SwerveDrive, controller: CommandPS4Controller):
        self.drive = drive
        self.controller = controller
        self.addRequirements(drive)

    def execute(self) -> None:
        self.drive.teleopDrive(
            ChassisSpeeds(
                DriveConstants.MAX_LINEAR_VELOCITY *
                DefaultDriveCommand.modifyAxis(-self.controller.getLeftY(), 0.1),
                DriveConstants.MAX_LINEAR_VELOCITY *
                DefaultDriveCommand.modifyAxis(-self.controller.getLeftX(), 0.1),
                DriveConstants.MAX_ANGULAR_VELOCITY *
                DefaultDriveCommand.modifyAxis(-self.controller.getRightX(), 0.1),
            )
        )

    @staticmethod
    def modifyAxis(joystickValue: float, deadband: float) -> float:
        # Deadband
        if abs(joystickValue) < deadband:
            return 0.0
        # Square the axis for finer control at low speeds
        return (joystickValue / abs(joystickValue)) * (joystickValue ** 2)