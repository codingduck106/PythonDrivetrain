import math
from phoenix6.swerve import (
    SwerveDrivetrain, 
    SwerveDrivetrainConstants,
    SwerveModuleConstantsFactory,
    ClosedLoopOutputType, 
    SteerFeedbackType,
    SwerveModule
)
from wpimath.geometry import Pose2d, Rotation2d, Rotation3d
from phoenix6.hardware import TalonFX, CANcoder
from phoenix6.configs import TalonFXConfiguration, CANcoderConfiguration, CurrentLimitsConfigs
from phoenix6.units import rotations_per_second, rotation
from commands2 import Subsystem
from phoenix6.configs import Slot0Configs
from phoenix6.swerve.requests import FieldCentric, ChassisSpeeds, ApplyRobotSpeeds, SwerveModuleState, FieldCentricFacingAngle
from subsystems.simdrivetrain import SimDrivetrain
import wpilib
from wpilib import SmartDashboard
from ntcore import NetworkTableInstance
from constants import *

kMaxSpeed = 6.0  # m/s
kMaxAngularSpeed = 2 * math.pi  # rad/sec
kWheelRadius = 0.0390398  # meters
kDriveGearRatio = 6.75
kTurnGearRatio = 150.0 / 7
FRONT_DIST = 0.4064
LEFT_DIST = 0.3302


class Drive(Subsystem):
    """The Drivetrain subsystem.

    Copied from Dr. Womp's vision testing, which was copied from maelstrom.

    Bask in its glory."""

    DEFAULT_MAX_VELOCITY_METERS_PER_SECOND = 6
    DEFAULT_MAX_ROTATIONS_PER_SECOND = 1.2
    WHEEL_RADIUS_IN = 1.537

    multiplier = 1

    FRONT_DIST = 0.4064
    LEFT_DIST = 0.3302

    max_radians_per_second = DEFAULT_MAX_ROTATIONS_PER_SECOND * math.pi * 2

    def __init__(self) -> None:
        """Initializes the drivetrain subsystem.
        
        It's a rare method that doesn't take anything or return anything. Then again, it's a constructor."""

        FLSteerOFFSET = -0.240234
        FRSteerOFFSET = -0.011475
        BLSteerOFFSET = -0.108887
        BRSteerOFFSET = -0.148348

        self.steerGains = (Slot0Configs() # The nsteering gains, aka tuning parameters for the PID controllers, which control the motors
                      .with_k_p(0)
                      .with_k_i(0)
                      .with_k_d(0)
                      .with_k_s(0.21041)
                      .with_k_v(2.68)
                      .with_k_a(0.084645))
        
        self.driveGains = (Slot0Configs() # The driving gains, aka tuning parameters for the PID controllers, which control the motors
                      .with_k_p(2.5)
                      .with_k_i(0)
                      .with_k_d(0)
                      .with_k_s(6.4111)
                      .with_k_v(0.087032)
                      .with_k_a(0))
        
        drivetrainConstants = SwerveDrivetrainConstants().with_pigeon2_id(GYRO).with_can_bus_name("swerve") # the constants for the drivetrain

        moduleConstantsFactory = (SwerveModuleConstantsFactory()  # A SwerveModule constant creator which, given some presets, can create the module constants for every swerve module
                                  .with_drive_motor_gear_ratio(kDriveGearRatio)
                                  .with_steer_motor_gear_ratio(kTurnGearRatio)
                                  .with_wheel_radius(self.WHEEL_RADIUS_IN * 0.0254)
                                  .with_slip_current(90)
                                  .with_steer_motor_gains(self.steerGains)
                                  .with_drive_motor_gains(self.driveGains)
                                  .with_drive_motor_closed_loop_output(ClosedLoopOutputType
                                                                       .TORQUE_CURRENT_FOC)
                                  .with_steer_motor_closed_loop_output(ClosedLoopOutputType.VOLTAGE)
                                  .with_speed_at12_volts(6)
                                  .with_feedback_source(SteerFeedbackType.FUSED_CANCODER)
                                  .with_coupling_gear_ratio(3.5)
                                  .with_drive_motor_initial_configs(TalonFXConfiguration())
                                  .with_steer_motor_initial_configs(TalonFXConfiguration())
                                  .with_encoder_initial_configs(CANcoderConfiguration()))
        
        frontLeft = moduleConstantsFactory.create_module_constants( # front left swerve module
            FLConstants.TURN, FLConstants.DRIVE, FLConstants.CAN,
            FLSteerOFFSET,
            FRONT_DIST, LEFT_DIST,
            False, True, False
        )
        frontRight = moduleConstantsFactory.create_module_constants( # front right swerve module
            FRConstants.TURN, FRConstants.DRIVE, FRConstants.CAN,
            FRSteerOFFSET,
            FRONT_DIST, -LEFT_DIST,
            False, True, False
        )
        backLeft = moduleConstantsFactory.create_module_constants( # back left swerve module
            BLConstants.TURN, BLConstants.DRIVE, BLConstants.CAN,
            BLSteerOFFSET,
            -FRONT_DIST, LEFT_DIST,
            False, True, False
        )
        backRight = moduleConstantsFactory.create_module_constants( # back right swerve module
            BRConstants.TURN, BRConstants.DRIVE, BRConstants.CAN,
            BRSteerOFFSET,
            -FRONT_DIST, -LEFT_DIST,
            False, True, False
        )
        
        self.drivetrain = SwerveDrivetrain(TalonFX, TalonFX, CANcoder, drivetrainConstants, [frontLeft, frontRight, backLeft, backRight]) # Creates a drivetrain object!

        # ===================================LOGGING===================================#

        network_table = NetworkTableInstance.getDefault().getTable("Drive")
        self.expected_state_publisher = network_table.getStructArrayTopic("expected state", SwerveModuleState).publish()
        self.actual_state_publisher = network_table.getStructArrayTopic("actual state", SwerveModuleState).publish()
        self.current_pose_publisher = network_table.getStructTopic("current pose", Pose2d).publish()
        self.module_positions_publisher = network_table.getDoubleArrayTopic("module positions").publish()
        self.angle_publisher = network_table.getStructTopic("pigeon angle", Rotation3d).publish()
    
        #===================================PID TUNING==================================#

        # ==================== DRIVE PID + FF ====================
        SmartDashboard.putNumber("drive_kP", 2.5)
        SmartDashboard.putNumber("drive_kI", 0.0)
        SmartDashboard.putNumber("drive_kD", 0.0)
        SmartDashboard.putNumber("drive_kS", 6.4111)
        SmartDashboard.putNumber("drive_kV", 0.087032)
        SmartDashboard.putNumber("drive_kA", 0.0)

        # ==================== STEER PID + FF ====================
        SmartDashboard.putNumber("steer_kP", 0.0)
        SmartDashboard.putNumber("steer_kI", 0.0)
        SmartDashboard.putNumber("steer_kD", 0.0)
        SmartDashboard.putNumber("steer_kS", 0.21041)
        SmartDashboard.putNumber("steer_kV", 2.68)
        SmartDashboard.putNumber("steer_kA", 0.084645)


        for i in range(4): # applies a configuration to every drive motor on the drivetrain
            (self.drivetrain.get_module(i)
             .drive_motor
             .configurator.apply(CurrentLimitsConfigs()
                                 .with_supply_current_limit(60)
                                 .with_supply_current_limit_enable(True)
                                 .with_stator_current_limit_enable(False)))
            

        # creates a simulation drivetrain if in simulation mode            
        if wpilib.RobotBase.isSimulation():
            self.sim_drivetrain = SimDrivetrain(network_table, self.drivetrain, [frontLeft, frontRight, backLeft, backRight])
        else:
            self.sim_drivetrain = None

    @staticmethod
    def modifyAxis(value: float) -> float:
        """Applies deadband and inversion to a joystick input.
        
        :param value: `float` value representing the joystick's input
        :returns: `float` value containing the modified joystick input."""
        if abs(value) <= 0.07:
            value = 0
        value = math.copysign(value*value, value)
        return value
    

    
    # some instance variables i forgot to define in main
    apply_robot_speeds_applier = ApplyRobotSpeeds()
    field_centric_facing_angle_applier = FieldCentricFacingAngle()
    field_centric_applier = FieldCentric().with_drive_request_type(SwerveModule.DriveRequestType.VELOCITY)

    def on_red(self) -> bool:
        """Returns whether or not the robot's current Alliance is `RED` or not
        
        :returns: `bool` whether the robot's current alliance is `RED`"""
        return wpilib.DriverStation.getAlliance() == wpilib.DriverStation.Alliance.kRed
    
    # ANOTHER instance variable i forgot to define lol
    correct_rotation = False

    def drive(self, xSpeed: float, ySpeed: float, rotation: float) -> None:
        """Drives the robot given the joystick values.
        
        Drives field-relative, not robot relative.
        
        :param xSpeed: `float` joystick value for the xSpeed
        :param ySpeed: `float` joystick value for the ySpeed
        :param rotation: `float` joystick value for the rotation"""
        multiplier = -self.multiplier if self.on_red() and self.correct_rotation else self.multiplier # whether or not too invert the joysticks
        self.drivetrain.set_control(
            self.field_centric_applier # the field centric drive request
            .with_velocity_x(xSpeed * multiplier) # the x speed, should be obvious
            .with_velocity_y(ySpeed * multiplier) # the y speed, should be obvious
            .with_rotational_rate(
                rotation * self.multiplier # the rotational rate, should be obvious
            )
        )

    def driveRobotRelative(self, demand: ChassisSpeeds) -> None:
        """Drives the robot relative to itself
        
        Used by Pathplanner to run autos.
        
        :param demand: `ChassisSpeeds` object containing the speeds for the robot"""
        self.drivetrain.set_control(self.apply_robot_speeds_applier.with_speeds(demand))

    def turn_to_face(self, rotation: Rotation2d) -> None:
        """Turns the robot in a given direction
        
        :param rotation: `Rotation2d` object containing the rotation to turn to"""
        self.drivetrain.set_control(self.field_centric_facing_angle_applier.with_target_direction(rotation))

    def set_rotation_velocity(self, rotation_rate: rotations_per_second) -> None:
        """Changes how fast the robot turns
        
        :param rotation_rate: `rotations_per_second`, the angular velocity at which the robot should turn."""
        self.drivetrain.set_control(self.field_centric_applier.with_rotational_rate(rotation_rate))
    
    def reset_pose(self) -> None:
        """Resets the robot's pose.

        One of those rare methods that don't take any arguments or return anything."""
        self.correct_rotation = False
        self.drivetrain.seed_field_centric()

    def set_pose(self, pose: Pose2d) -> None:
        """Sets either the simulation drivetrain's or the real drivetrain's pose to the given pose.
        
        :param pose: `Pose2d` object containing the Pose to set the drivetrain's pose to."""
        self.correct_rotation = True
        if pose is not None:
            self.drivetrain.reset_pose(pose)
            if self.sim_drivetrain is not None:
                self.sim_drivetrain.reset_pose(pose)
        else:
            wpilib.DriverStation.startDataLog("set_pose() was passed a none. probably not intended, but check your code again", False)
    
    def periodic(self) -> None:
        """Logs stuff.
        
        Again, one of those rare functions that don't take anything or return anything."""
        self.expected_state_publisher.set(self.drivetrain.get_state().module_targets)
        self.actual_state_publisher.set(self.drivetrain.get_state().module_states)
        rotation = self.get_rotation3d()
        self.angle_publisher.set(rotation)
        drive_pose = self.get_pose()
        self.current_pose_publisher.set(drive_pose)
        self.module_positions_publisher.set([self.get_position(i) for i in range(4)])

        self.update_pid_gains()

    def simulationPeriodic(self) -> None:
        """Runs the sim drivetrain's periodic function.
        
        Once again, one of those rare functions that don't take anything or return anything."""
        if self.sim_drivetrain is None:
            return

        self.sim_drivetrain.periodic()

    def enable_slow_mode(self, enable: bool) -> None:
        """Enables slow mode, makes the robot's motors move at 30% the speed.
        
        :param enable: `bool` object, whether to enable slow mode or not. True if yes, False if no."""
        self.multiplier = 0.3 if enable else 1
    
    def close(self) -> None:
        """Closes the drivetrain instance.
        
        ONCE again, a rare method that doesn't take anything or return anything.
        
        I guess they aren't that rare though..."""
        self.drivetrain.close()

    #============================GETTERS======================================

    def get_robot_relative_speeds(self):
        """Gets the robot's speeds relative to itself.
        
        :returns: `ChassisSpeeds` object containing the robot's speeds."""
        return self.drivetrain.kinematics.toChassisSpeeds(tuple(self.drivetrain.get_state().module_states)) # type: ignore , because we know that the amount of modules the drivetrain has is always 4.
    
    def get_pose(self) -> Pose2d:
        """Gets the current pose of the robot.
        
        :returns: `Pose2d` object containing the current pose of the robot"""
        return self.drivetrain.get_state().pose
    
    def get_rotation3d(self) -> Rotation3d:
        """Gets the 3 dimensional rotation of the robot on the field, from the pigeon 2
        
        :returns: `Rotation3d` object containing the rotation from the pigeon2's quaternion values"""
        if self.sim_drivetrain is not None:
            return self.sim_drivetrain.get_rotation3d()
        return self.drivetrain.get_rotation3d()
    
    def get_position(self, module_id: int) -> rotation:
        """Gets a given swerve module's 'position', or the angle of the swerve module's wheel.
        
        :param module_id: `int`, the swerve module's id on the drivetrain, ranging from 0-3
        :returns: `rotation` object containing the swerve module's position"""
        return (self.drivetrain
                .get_module(module_id)
                .encoder
                .get_absolute_position()
                .value)
    
    #=================================PID TUNING==============================================
    def update_pid_gains(self):
        # DRIVE gains
        drive_kP = SmartDashboard.getNumber("drive_kP", 2.5)
        drive_kI = SmartDashboard.getNumber("drive_kI", 0.0)
        drive_kD = SmartDashboard.getNumber("drive_kD", 0.0)
        drive_kS = SmartDashboard.getNumber("drive_kS", 6.4111)
        drive_kV = SmartDashboard.getNumber("drive_kV", 0.087032)
        drive_kA = SmartDashboard.getNumber("drive_kA", 0.0)

        # STEER gains
        steer_kP = SmartDashboard.getNumber("steer_kP", 0.0)
        steer_kI = SmartDashboard.getNumber("steer_kI", 0.0)
        steer_kD = SmartDashboard.getNumber("steer_kD", 0.0)
        steer_kS = SmartDashboard.getNumber("steer_kS", 0.21041)
        steer_kV = SmartDashboard.getNumber("steer_kV", 2.68)
        steer_kA = SmartDashboard.getNumber("steer_kA", 0.084645)

        # Build configs
        self.driveGains = (
            Slot0Configs()
            .with_k_p(drive_kP)
            .with_k_i(drive_kI)
            .with_k_d(drive_kD)
            .with_k_s(drive_kS)
            .with_k_v(drive_kV)
            .with_k_a(drive_kA)
        )

        self.steerGains = (
            Slot0Configs()
            .with_k_p(steer_kP)
            .with_k_i(steer_kI)
            .with_k_d(steer_kD)
            .with_k_s(steer_kS)
            .with_k_v(steer_kV)
            .with_k_a(steer_kA)
        )

        # Apply to all 4 modules
        for i in range(4):
            module = self.drivetrain.get_module(i)
            module.drive_motor.configurator.apply(self.driveGains)
            module.steer_motor.configurator.apply(self.steerGains)
