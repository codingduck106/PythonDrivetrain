from pathplannerlib.config import RobotConfig, PIDConstants
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.controller import PPHolonomicDriveController


from drivetrain import *
from ntcore import NetworkTableInstance
from wpilib import SmartDashboard
from commands2.instantcommand import InstantCommand
from constants import *

class RobotContainer:
    def __init__(self):
        """Initializes a robot container"""
        self.drive = Drivetrain()
        self.configureBindings()

        config = RobotConfig.fromGUISettings()

        AutoBuilder.configure(
            self.drive.getPose,
            self.drive.resetPose,
            self.drive.getRobotRelativeSpeeds,
            self.drive.driveRobotRelative,
            PPHolonomicDriveController(
                PIDConstants(5.0, 0.0, 0.0),
                PIDConstants(5.0, 0.0, 0.0),
            ),
            config,
            self.drive.shouldFlipPath,
            self.drive,
        )

        # Auto chooser
        self.autoChooser = AutoBuilder.buildAutoChooser()

        

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
