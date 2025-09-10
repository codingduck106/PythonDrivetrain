from pathplannerlib.auto import PathPlannerAuto, AutoBuilder
from pathplannerlib.controller import PPHolonomicDriveController
from pathplannerlib.config import RobotConfig, PIDConstants
from wpilib import DriverStation, SmartDashboard
from commands2.button import CommandPS4Controller
from commands2.instantcommand import InstantCommand
from drivetrain import Drivetrain
from constants import *

# TODO: ONLY INCLUDES DRIVETRAIN. ADD OTHER SUBSYSTEMS LATER

class RobotContainer:
    ALLIANCE_USED_IN_PATHS = DriverStation.Alliance.kBlue
    drive: Drivetrain
    
    def __init__(self):
        self.drive = Drivetrain()
        self.configureBindings()
        self.autoChooser = AutoBuilder.buildAutoChooser()
        SmartDashboard.putData("Auto Chooser", self.autoChooser)
    def configureBindings(self):
        RESET_POSE.onTrue(InstantCommand(self.drive.resetPose, self.drive)) # type: ignore

    # def configureAuto(self, drive: Drivetrain):
    #     config = RobotConfig.fromGUISettings()
    #     AutoBuilder.configure(
    #         drive.getPose,
    #         drive.resetPose,
    #         drive.getRobotRelativeSpeeds,
    #         drive.driveRobotRelative,
    #         PPHolonomicDriveController(
    #             PIDConstants(15,0.0,0.0),
    #             PIDConstants(6.85,0.0,1.3)
    #         ),
    #         config,
    #         lambda: True if DriverStation.getAlliance() != self.ALLIANCE_USED_IN_PATHS else False,
    #         self.drive
    #     )
    #     return AutoBuilder.buildAutoChooser()
    
    def getAutonomousCommand(self):
        return self.autoChooser.getSelected()