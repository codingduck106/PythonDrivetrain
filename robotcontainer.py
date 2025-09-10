from pathplannerlib.auto import AutoBuilder
from drivetrain import Drivetrain
from ntcore import NetworkTableInstance
from wpilib import SmartDashboard, DriverStation
from commands2.instantcommand import InstantCommand
from constants import *

class RobotContainer:
    def __init__(self):
        self.drive = Drivetrain()
        self.configureBindings()

        # Auto chooser
        self.autoChooser = AutoBuilder.buildAutoChooser()

        # NetworkTables
        nt_instance = NetworkTableInstance.getDefault()
        self.nt_table = nt_instance.getTable("SmartDashboard")
        self.auto_topic = self.nt_table.getStringTopic("SelectedAuto").publish()
        SmartDashboard.putData("Auto Chooser", self.autoChooser)

    def configureBindings(self):
        RESET_POSE.onTrue(InstantCommand(self.drive.resetPose, self.drive))

    def getAutonomousCommand(self):
        cmd = self.autoChooser.getSelected()
        name = str(self.autoChooser.getSelected()) if cmd else "None"
        self.auto_topic.set(name)
        return cmd
