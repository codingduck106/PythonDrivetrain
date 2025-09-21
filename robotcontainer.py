from pathplannerlib.config import RobotConfig, PIDConstants
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.controller import PPHolonomicDriveController
from wpimath.units import meters
from subsystems.drivetrain import Drive
from wpimath.geometry import Pose2d, Rotation2d
import wpilib
from ntcore import NetworkTableInstance
from wpilib import SmartDashboard
from commands2.instantcommand import InstantCommand
from commands.defaultdrivecommand import DefaultDriveCommand
from constants import *
import math

DEFAULT_MAX_VELOCITY_METERS_PER_SECOND = 6
DEFAULT_MAX_ROTATIONS_PER_SECOND = 1.2
max_radians_per_second = DEFAULT_MAX_ROTATIONS_PER_SECOND * math.pi * 2


class RobotContainer:
    """Fixed robot container to prevent command conflicts"""

    ALLIANCE_USED_IN_PATHS= wpilib.DriverStation.Alliance.kBlue
    
    bot_pose_blue_origin = Pose2d(meters(-8.7736), meters(-4.0257), Rotation2d())
    def __init__(self, alliance: wpilib.DriverStation.Alliance | None):
        """Initialize with conflict prevention"""
        self.drive = Drive()  # Use the fixed drivetrain
        self.drive.setDefaultCommand(
            DefaultDriveCommand(self.drive,
                                lambda: -Drive.modifyAxis(DRIVER_CONTROLLER.getLeftY()) * DEFAULT_MAX_VELOCITY_METERS_PER_SECOND,
                                lambda: -Drive.modifyAxis(DRIVER_CONTROLLER.getLeftX()) * DEFAULT_MAX_VELOCITY_METERS_PER_SECOND,
                                lambda: -Drive.modifyAxis(DRIVER_CONTROLLER.getRightX()) * max_radians_per_second)
        )
        self.configureBindings()
        
        self.autoChooser = self.configure_auto(self.drive)
        SmartDashboard.putData("Auto Routine", self.autoChooser)
        self.configureBindings()
        # Reduced NetworkTables usage
        self.nt_table = NetworkTableInstance.getDefault().getTable("SmartDashboard")
        # if self.autoChooser:
        #     SmartDashboard.putData("Auto Chooser", self.autoChooser)
        # self.DRIVE_SYSID = SwerveSysidRequest(MotorType.Drive, RequestType.torque_current_foc)
        # self.STEER_SYSID = SwerveSysidRequest(MotorType.Swerve, RequestType.voltage_out)

    def configure_auto(self, drive: Drive): # add more subsystems later
        try:
            config = RobotConfig.fromGUISettings()
        except Exception as e:
            raise RuntimeError("Could not get Config", e)
        
        AutoBuilder.configure(
                drive.get_pose,
                drive.set_pose,
                drive.get_robot_relative_speeds,
                lambda speeds, ff: drive.driveRobotRelative(speeds),  # This calls the fixed method
                PPHolonomicDriveController(
                    PIDConstants(15.0, 0.0, 0.0),
                    PIDConstants(6.85, 0.0, 1.3),
                ),
                config,
                lambda: wpilib.DriverStation.getAlliance() != self.ALLIANCE_USED_IN_PATHS,
                drive,
            )
        self.configure_auto_commands() # add subsystems as arguments later
        return AutoBuilder.buildAutoChooser()

    # def get_sysid_routines(self, registry: SubsystemRegistry):
    #     routines = []
    #     routines.append(
    #         DropdownEntry("Drive-Drive Motor",
    #                       SysIdRoutine(
    #                           SysIdRoutine.Config(
    #                               0,
    #                               0,
    #                               0,
    #                               lambda s: SignalLogger.write_string("state", s.__str__()) # type: ignore
    #                           ),
    #                       SysIdRoutine.Mechanism(
    #                           lambda v: (registry
    #                                      .subsystems[type(Drivetrain)]
    #                                      .run(self.DRIVE_SYSID.with_voltage)
    #                                      .schedule()),
    #                             lambda s: None,
    #                             registry.subsystems[Drivetrain]
    #                       )
    #                       ))
    #     )
        # routines.append(
        #     DropdownEntry("Drive-Steer Motor",
        #                   SysIdRoutine(
        #                       SysIdRoutine.Config(
        #                           0,
        #                           0,
        #                           0,
        #                           lambda s: SignalLogger.write_string("state", s.__str__()) # type: ignore
        #                       ),
        #                   SysIdRoutine.Mechanism(
        #                       lambda v: (registry
        #                                  .subsystems[Drivetrain]),
        #                         lambda s: None,
        #                         registry.subsystems[Drivetrain]
        #                   )
        #                   ))
        # )

    @classmethod
    def to_bot_pose_blue(cls, orig: Pose2d):
        return orig.relativeTo(cls.bot_pose_blue_origin)
    
    
    def configure_auto_commands(self):
        pass # leave empty for now, add subsystem functions later
    
    def configureBindings(self):
        """Simplified bindings to prevent conflicts"""
        
        # Only bind essential commands
        RESET_POSE.onTrue(InstantCommand(lambda: self.drive.reset_pose()))
    
    def getAutonomousCommand(self):
        """Get auto command with error handling"""
        return self.autoChooser.getSelected()
    
    def close(self): # add more subsystems later
        self.drive.close()
