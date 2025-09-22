from pathplannerlib.config import RobotConfig, PIDConstants
from pathplannerlib.auto import AutoBuilder, SendableChooser
from pathplannerlib.controller import PPHolonomicDriveController
from wpimath.units import meters
from subsystems import Drive
from wpilib import RobotBase
from wpimath.geometry import Pose2d, Rotation2d
import wpilib
from commands2 import Command
from wpilib import SmartDashboard
from commands2.instantcommand import InstantCommand
from commands import DefaultDriveCommand
from constants import *
import math
import typing

DEFAULT_MAX_VELOCITY_METERS_PER_SECOND = 6
DEFAULT_MAX_ROTATIONS_PER_SECOND = 1.2
max_radians_per_second = DEFAULT_MAX_ROTATIONS_PER_SECOND * math.pi * 2


class RobotContainer:
    """Fixed robot container to prevent command conflicts"""

    ALLIANCE_USED_IN_PATHS= wpilib.DriverStation.Alliance.kBlue
    
    bot_pose_blue_origin = Pose2d(meters(-8.7736), meters(-4.0257), Rotation2d())
    def __init__(self):
        """Initializes the Robot Container, which is a container containing every subsystem the bot has."""
        self.drive = Drive()  # Use the fixed drivetrain
        self.drive.setDefaultCommand(
            DefaultDriveCommand(self.drive,
                                lambda: -Drive.modifyAxis(DRIVER_CONTROLLER.getLeftY()) * DEFAULT_MAX_VELOCITY_METERS_PER_SECOND, # SEE, LAMBDA FUNCTIONS! I TOLD YOU! (if you don't understand, check defaultdrivecommand.py under the commands fodler)
                                lambda: -Drive.modifyAxis(DRIVER_CONTROLLER.getLeftX()) * DEFAULT_MAX_VELOCITY_METERS_PER_SECOND,
                                lambda: -Drive.modifyAxis(DRIVER_CONTROLLER.getRightX()) * max_radians_per_second)
        )

        self.configureBindings() # configures the controller bindings for the subsystems.
        
        self.autoChooser = self.configure_auto(self.drive) # configures the auto.
        SmartDashboard.putData("Auto Routine", self.autoChooser) # adds an autochooser to SmartDashboard

    def configure_auto(self, drive: Drive) -> SendableChooser: # add more subsystems later
        """Configures the auto. We initialize Pathplanner's auto stuff here.
        
        Usually contains other subsystems, but we only have drivetrain.
        
        :param drive: the drivetrain instance.
        :returns: The autochooser."""
        try:
            config = RobotConfig.fromGUISettings() # gets the Robot config from a json file
        except Exception as e:
            raise RuntimeError("Could not get Config", e)
        
        AutoBuilder.configure( # AUTOBUILDER CONFIGURATION RAHH
                drive.get_pose,
                drive.set_pose,
                drive.get_robot_relative_speeds,
                lambda speeds, ff: drive.driveRobotRelative(speeds),  # We use a lambda here because pathplanner also gives us feedforwards, but we don't use feedforwards.
                PPHolonomicDriveController(
                    PIDConstants(15.0, 0.0, 0.0),
                    PIDConstants(6.85, 0.0, 1.3),
                ),#
                config,
                lambda: wpilib.DriverStation.getAlliance() != self.ALLIANCE_USED_IN_PATHS, # whether pathplanner should flip the autopaths or not depending on which alliance we're on. True if we're on red.
                drive,
        )
        self.configure_auto_commands() # add subsystems as arguments later
        return AutoBuilder.buildAutoChooser("blue-left-leave")

    @classmethod
    def to_bot_pose_blue(cls, orig: Pose2d) -> Pose2d:
        """In Dr. Womp, this is a static method.
        
        We use class method here because we want to actually modify the class variable.
        
        :param orig: the origin of the field.
        :returns: a `Pose2d` object containing the current pose relative to the origin."""
        return orig.relativeTo(cls.bot_pose_blue_origin)
    
    
    def configure_auto_commands(self) -> None:
        """We add other subsystems here. They have auto commands too!"""
        pass # leave empty for now, add subsystem functions later
    
    def configureBindings(self) -> None:
        """Configures the controller bindings. So far, we only have drive, which is this."""
        if not RobotBase.isSimulation():
            RESET_POSE.onTrue(InstantCommand(lambda: self.drive.reset_pose()))
        else:
            pass
    
    def getAutonomousCommand(self) -> Command | typing.Any: #================GETTER=====================
        """Gets the auto command"""
        return self.autoChooser.getSelected()
    
    def close(self): # add more subsystems later
        """closes all the subsystems.
        
        so far, only drive :("""
        self.drive.close()
