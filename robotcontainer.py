from pathplannerlib.auto import AutoBuilder, RobotConfig
from pathplannerlib.controller import PPHolonomicDriveController
from pathplannerlib.config import PIDConstants
from drivetrain import *
from ntcore import NetworkTableInstance
from wpilib import SmartDashboard, DriverStation
from commands2.instantcommand import InstantCommand
from constants import *

class RobotContainer:
    def __init__(self):
        """Initializes a robot container"""
        self.drive = Drivetrain()
        self.configureBindings()

        # Auto chooser
        self.autoChooser = AutoBuilder.buildAutoChooser()

        # AutoBuilder
        config = RobotConfig.fromGUISettings()
        AutoBuilder.configure(
            self.drive.getPose,
            self.drive.resetPose,
            self.drive.getRobotRelativeSpeeds,
            lambda speeds, ff: self.drive.driveRobotRelative(speeds, ff),
            PPHolonomicDriveController(
                PIDConstants(5.0, 0.0, 0.0),
                PIDConstants(5.0, 0.0, 0.0),
            ),
            config,
            self.drive.shouldFlipPath,
            self.drive,
        )

        # NetworkTables
        nt_instance = NetworkTableInstance.getDefault()
        self.nt_table = nt_instance.getTable("SmartDashboard")
        self.auto_topic = self.nt_table.getStringTopic("SelectedAuto").publish()
        SmartDashboard.putData("Auto Chooser", self.autoChooser)

    def configureBindings(self):
        """Configure controller bindings"""
        RESET_POSE.onTrue(InstantCommand(self.drive.resetPose, self.drive))

    def getAutonomousCommand(self):
        """gets the autonomous command from the autoChooser"""
        cmd = self.autoChooser.getSelected()
        name = str(self.autoChooser.getSelected()) if cmd else "None"
        self.auto_topic.set(name)
        return cmd
