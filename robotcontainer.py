from pathplannerlib.auto import AutoBuilder
from drivetrain import Drivetrain
from ntcore import NetworkTableInstance
from wpilib import SmartDashboard, DriverStation
from commands2.instantcommand import InstantCommand
from constants import *
from elevator import Elevator

class RobotContainer:
    def __init__(self):
        """Initializes a robot container"""
        self.drive = Drivetrain()
        # Initialize the elevator with master/follower IDs
        self.elevator = Elevator(ELEVATOR_1, ELEVATOR_2)
        self.configureBindings()

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
