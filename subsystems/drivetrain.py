import math
from phoenix6.swerve import (
    SwerveDrivetrain, 
    SwerveDrivetrainConstants,
    SwerveModuleConstantsFactory,
    ClosedLoopOutputType, 
    SteerFeedbackType
)
from wpimath.geometry import Pose2d, Rotation2d, Rotation3d
from phoenix6.hardware import TalonFX, CANcoder, Pigeon2
from phoenix6.configs import TalonFXConfiguration, CANcoderConfiguration, CurrentLimitsConfigs
from commands2 import Subsystem, Command
from phoenix6.controls import StaticBrake
from phoenix6.configs import Slot0Configs
from phoenix6.swerve.requests import *
from subsystems.simdrivetrain import SimDrivetrain
import wpilib
# from structs import SwerveModuleStateStruct, Pose2dStruct, Rotation3dStruct
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
    DEFAULT_MAX_VELOCITY_METERS_PER_SECOND = 6
    DEFAULT_MAX_ROTATIONS_PER_SECOND = 1.2
    WHEEL_RADIUS_IN = 1.537

    multiplier = 1

    FRONT_DIST = 0.4064
    LEFT_DIST = 0.3302

    max_radians_per_second = DEFAULT_MAX_ROTATIONS_PER_SECOND * math.pi * 2

    def __init__(self):
        FLSteerOFFSET = -0.240234
        FRSteerOFFSET = -0.011475
        BLSteerOFFSET = -0.108887
        BRSteerOFFSET = -0.148348

        steerGains = (Slot0Configs()
                      .with_k_p(50)
                      .with_k_i(0)
                      .with_k_d(3.0889)
                      .with_k_s(0.21041)
                      .with_k_v(2.68)
                      .with_k_a(0.084645))
        
        driveGains = (Slot0Configs()
                      .with_k_p(2.5)
                      .with_k_i(0)
                      .with_k_d(0)
                      .with_k_s(6.4111)
                      .with_k_v(0.087032)
                      .with_k_a(0))
        
        drivetrainConstants = SwerveDrivetrainConstants().with_pigeon2_id(GYRO).with_can_bus_name("swerve")

        moduleConstantsFactory = (SwerveModuleConstantsFactory()
                                  .with_drive_motor_gear_ratio(kDriveGearRatio)
                                  .with_steer_motor_gear_ratio(kTurnGearRatio)
                                  .with_wheel_radius(self.WHEEL_RADIUS_IN)
                                  .with_slip_current(90)
                                  .with_steer_motor_gains(steerGains)
                                  .with_drive_motor_gains(driveGains)
                                  .with_drive_motor_closed_loop_output(ClosedLoopOutputType
                                                                       .TORQUE_CURRENT_FOC)
                                  .with_steer_motor_closed_loop_output(ClosedLoopOutputType.VOLTAGE)
                                  .with_speed_at12_volts(6)
                                  .with_feedback_source(SteerFeedbackType.FUSED_CANCODER)
                                  .with_coupling_gear_ratio(3.5)
                                  .with_drive_motor_initial_configs(TalonFXConfiguration())
                                  .with_steer_motor_initial_configs(TalonFXConfiguration())
                                  .with_encoder_initial_configs(CANcoderConfiguration()))
        
        frontLeft = moduleConstantsFactory.create_module_constants(
            FLConstants.TURN, FLConstants.DRIVE, FLConstants.CAN,
            FLSteerOFFSET,
            FRONT_DIST, LEFT_DIST,
            False, True, False
        )
        frontRight = moduleConstantsFactory.create_module_constants(
            FRConstants.TURN, FRConstants.DRIVE, FRConstants.CAN,
            FRSteerOFFSET,
            FRONT_DIST, -LEFT_DIST,
            False, True, False
        )
        backLeft = moduleConstantsFactory.create_module_constants(
            BLConstants.TURN, BLConstants.DRIVE, BLConstants.CAN,
            BLSteerOFFSET,
            -FRONT_DIST, LEFT_DIST,
            False, True, False
        )
        backRight = moduleConstantsFactory.create_module_constants(
            BRConstants.TURN, BRConstants.DRIVE, BRConstants.CAN,
            BRSteerOFFSET,
            -FRONT_DIST, -LEFT_DIST,
            False, True, False
        )
        
        self.drivetrain = SwerveDrivetrain(TalonFX, TalonFX, CANcoder, drivetrainConstants, [frontLeft, frontRight, backLeft, backRight])

        network_table = NetworkTableInstance.getDefault().getTable("Drive")
        self.expected_state_publisher = network_table.getStructArrayTopic("expected state", SwerveModuleState).publish()
        self.actual_state_publisher = network_table.getStructArrayTopic("actual state", SwerveModuleState).publish()
        self.current_pose_publisher = network_table.getStructTopic("current pose", Pose2d).publish()
        self.module_positions_publisher = network_table.getDoubleArrayTopic("module positions").publish()
        self.angle_publisher = network_table.getStructTopic("pigeon angle", Rotation3d).publish()

        for i in range(4):
            (self.drivetrain.get_module(i)
             .drive_motor
             .configurator.apply(CurrentLimitsConfigs()
                                 .with_supply_current_limit(60)
                                 .with_supply_current_limit_enable(True)
                                 .with_stator_current_limit_enable(False)))
            
        if wpilib.RobotBase.isSimulation():
            self.sim_drivetrain = SimDrivetrain(network_table, self.drivetrain, [frontLeft, frontRight, backLeft, backRight])
        else:
            self.sim_drivetrain = None

    @staticmethod
    def modifyAxis(value: float):
        if abs(value) <= 0.1:
            value = 0
        value = math.copysign(value*value, value)
        return value

    # def create_default_command(self):
    #     return (
    #         self,
    #         lambda: -self.modifyAxis(DRIVER_CONTROLLER.getLeftY()) * self.DEFAULT_MAX_VELOCITY_METERS_PER_SECOND,
    #         lambda: -self.modifyAxis(DRIVER_CONTROLLER.getLeftX()) * self.DEFAULT_MAX_VELOCITY_METERS_PER_SECOND,
    #         lambda: -self.modifyAxis(DRIVER_CONTROLLER.getRightX()) * self.max_radians_per_second
    #     )

    def get_rotation3d(self):
        if self.sim_drivetrain is not None:
            return self.sim_drivetrain.get_rotation3d()
        return self.drivetrain.get_rotation3d()
    
    def get_position(self, module_id: int):
        return rotation(self.drivetrain
                .get_module(module_id)
                .encoder
                .get_absolute_position()
                .value)
    
    apply_robot_speeds_applier = ApplyRobotSpeeds()
    field_centric_facing_angle_applier = FieldCentricFacingAngle()
    field_centric_applier = FieldCentric().with_drive_request_type(SwerveModule.DriveRequestType.VELOCITY)

    def on_red(self):
        return wpilib.DriverStation.getAlliance() == wpilib.DriverStation.Alliance.kRed
    
    correct_rotation = False

    def drive(self, xSpeed: float, ySpeed: float, rotation: float):
        multiplier = -self.multiplier if self.on_red() and self.correct_rotation else self.multiplier
        self.drivetrain.set_control(
            self.field_centric_applier
            .with_velocity_x(xSpeed * multiplier)
            .with_velocity_y(ySpeed * multiplier)
            .with_rotational_rate(
                rotation * self.multiplier
            )
        )

    def driveRobotRelative(self, demand: ChassisSpeeds):
        self.drivetrain.set_control(self.apply_robot_speeds_applier.with_speeds(demand))

    def turn_to_face(self, rotation: Rotation2d):
        self.drivetrain.set_control(self.field_centric_facing_angle_applier.with_target_direction(rotation))

    def set_rotation_velocity(self, rotation_rate: rotations_per_second):
        self.drivetrain.set_control(self.field_centric_applier.with_rotational_rate(rotation_rate))

    def get_pose(self):
        return self.drivetrain.get_state().pose
    
    def reset_pose(self):
        self.correct_rotation = False
        self.drivetrain.seed_field_centric()

    def set_pose(self, pose: Pose2d):
        self.correct_rotation = True
        if pose is not None:
            self.drivetrain.reset_pose(pose)
            if self.sim_drivetrain is not None:
                self.sim_drivetrain.reset_pose(pose)
        else:
            raise Exception("set_pose() was passed a none. probably not intended, but check your code again", False)
        
    def get_robot_relative_speeds(self):
        return self.drivetrain.kinematics.toChassisSpeeds(tuple(self.drivetrain.get_state().module_states)) # type: ignore
    
    def periodic(self):
        self.expected_state_publisher.set(self.drivetrain.get_state().module_targets)
        self.actual_state_publisher.set(self.drivetrain.get_state().module_states)
        rotation = self.get_rotation3d()
        self.angle_publisher.set(rotation)
        drive_pose = self.get_pose()
        self.current_pose_publisher.set(drive_pose)
        self.module_positions_publisher.set([self.get_position(i) for i in range(4)])

    def simulationPeriodic(self):
        if self.sim_drivetrain is None:
            return

        self.sim_drivetrain.periodic()

    def enable_slow_mode(self, enable: bool):
        multiplier = 0.3 if enable else 1
    
    def close(self):
        self.drivetrain.close()