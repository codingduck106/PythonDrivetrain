from ntcore import NetworkTableInstance
from wpilib import SmartDashboard
from commands2 import Command
from pathplannerlib.auto import AutoBuilder

class RobotContainer:

    def __init__(self, ntInstance: NetworkTableInstance):

        self.autopath_dropdown = AutoBuilder.buildAutoChooser()

        SmartDashboard.putData("Autopath selection", self.autopath_dropdown)

        self.configure_bindings()


    def configure_bindings(self):
        pass

    def get_auton_command(self) -> Command:
        return self.autopath_dropdown.getSelected()