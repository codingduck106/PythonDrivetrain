import math
import wpimath.geometry
from phoenix6.swerve import SwerveModule, SwerveDrivetrain, SwerveDrivetrainConstants, SimSwerveDrivetrain, SwerveModuleConstants
import wpimath.kinematics
from pathplannerlib.trajectory import DriveFeedforwards
from wpimath.kinematics import SwerveModuleState
from phoenix6.hardware import TalonFX, CANcoder, Pigeon2
from phoenix6.controls import StaticBrake
from phoenix6.configs import TalonFXConfiguration, CANcoderConfiguration
from constants import *
from commands2 import Subsystem
import wpilib
from ntcore import NetworkTableInstance

kMaxSpeed = 3.0  # meters/sec
kMaxAngularSpeed = math.pi  # rad/sec
kWheelRadius = 0.053975  # measured in meters
kDriveGearRatio = 6.75  # Motor rotations per wheel rotation
kTurnGearRatio = 12.8   # Motor rotations per module rotation
kModuleMaxAngularVelocity = math.pi
kModuleMaxAngularAcceleration = math.tau

class Drivetrain(Subsystem):
    """
    Drivetrain subsystem class for a swerve drive robot.
    Handles module initialization, kinematics, odometry, telemetry, and driving.
    """

    def __init__(self) -> None:
        """
        Initializes the drivetrain subsystem.

        Sets up swerve modules, gyro, kinematics, odometry, and NetworkTables telemetry.
        """
        # Module positions
        self.frontLeftLocation = wpimath.geometry.Translation2d(0.44, 0.44)
        self.frontRightLocation = wpimath.geometry.Translation2d(0.44, -0.44)
        self.backLeftLocation = wpimath.geometry.Translation2d(-0.44, 0.44)
        self.backRightLocation = wpimath.geometry.Translation2d(-0.44, -0.44)


        # Use Pigeon2 instead of AnalogGyro for better performance
        self.gyro = Pigeon2(GYRO, "swerve")
        self.sim_gyro = self.gyro.sim_state

        # Swerve drivetrain constants
        self.drivetrain_constants = (SwerveDrivetrainConstants()
                                              .with_can_bus_name("swerve")
                                                .with_pigeon2_id(self.gyro.device_id))
        self.drivetrainC = SwerveDrivetrain(TalonFX, TalonFX, CANcoder,
                                             self.drivetrain_constants,
                                                [
                                                    get_module_constants(FLConstants.DRIVE, FLConstants.TURN, FLConstants.CAN),
                                                    get_module_constants(FRConstants.DRIVE, FRConstants.TURN, FRConstants.CAN),
                                                    get_module_constants(BLConstants.DRIVE, BLConstants.TURN, BLConstants.CAN),
                                                    get_module_constants(BRConstants.DRIVE, BRConstants.TURN, BRConstants.CAN)
                                                ])
        self.frontLeft, self.frontRight, self.backLeft, self.backRight = [module for module in self.drivetrainC.modules]

        # Simulation drivetrain
        self.sim_drivetrain = SimSwerveDrivetrain([self.frontLeftLocation,
                                                    self.frontRightLocation,
                                                    self.backLeftLocation,
                                                    self.backRightLocation],
                                                   self.sim_gyro,
                                                     [
                                                    get_module_constants(FLConstants.DRIVE, FLConstants.TURN, FLConstants.CAN),
                                                    get_module_constants(FRConstants.DRIVE, FRConstants.TURN, FRConstants.CAN),
                                                    get_module_constants(BLConstants.DRIVE, BLConstants.TURN, BLConstants.CAN),
                                                    get_module_constants(BRConstants.DRIVE, BRConstants.TURN, BRConstants.CAN)
                                                    ])

        # Kinematics and odometry
        self.kinematics = wpimath.kinematics.SwerveDrive4Kinematics(
            self.frontLeftLocation,
            self.frontRightLocation,
            self.backLeftLocation,
            self.backRightLocation,
        )
        self.odometry = wpimath.kinematics.SwerveDrive4Odometry(
            self.kinematics,
            self.getRotation2d(),
            tuple([
                module.get_position(True) for module in self.drivetrainC.modules
            ]),  # type: ignore
        )

        # NetworkTables telemetry
        nt_instance = NetworkTableInstance.getDefault()
        self.nt_table = nt_instance.getTable("SmartDashboard")
        self.pose_topic = self.nt_table.getDoubleArrayTopic("DrivetrainPose").publish()
        self.module_states_topic = self.nt_table.getDoubleArrayTopic("DrivetrainModuleStates").publish()

    # ---------------------- Basic Methods ----------------------
    def getRotation2d(self) -> wpimath.geometry.Rotation2d:
        """
        Get the current rotation of the robot from the gyro.

        :returns: `Rotation2d` object representing current robot heading
        """
        return self.drivetrainC.get_rotation3d().toRotation2d()

    def getPose(self) -> wpimath.geometry.Pose2d:
        """
        Get the current pose of the robot.

        :returns: `Pose2d` object representing current position and orientation
        """
        return self.drivetrainC.get_state().pose

    def shouldFlipPath(self) -> bool:
        """
        Determine whether autonomous paths should be flipped based on alliance color.

        :returns: True if alliance is red, False otherwise
        """
        return wpilib.DriverStation.getAlliance() == wpilib.DriverStation.Alliance.kRed

    def resetPose(self, pose: wpimath.geometry.Pose2d | None = None) -> None:
        """
        Reset the robot's odometry to a specified pose.

        :param pose: `Pose2d` object to reset to. Defaults to (0,0,0) if None.
        """
        if pose is None:
            pose = wpimath.geometry.Pose2d()
        self.drivetrainC.reset_pose(pose)

    # ---------------------- Driving ----------------------
    def drive(self, xSpeed, ySpeed, rot, fieldRelative: bool, periodSeconds: float):
        """
        Drive the robot given speeds and rotation.

        :param xSpeed: x velocity in meters/sec
        :param ySpeed: y velocity in meters/sec
        :param rot: rotational speed in radians/sec
        :param fieldRelative: True if speeds are field-relative
        :param periodSeconds: loop time for discretization
        """
        if fieldRelative:
            chassisSpeeds = wpimath.kinematics.ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeed, ySpeed, rot, self.getRotation2d()
            )
        else:
            chassisSpeeds = wpimath.kinematics.ChassisSpeeds(xSpeed, ySpeed, rot)

        swerveStates = self.drivetrainC.kinematics.toSwerveModuleStates(
            wpimath.kinematics.ChassisSpeeds.discretize(chassisSpeeds, periodSeconds)
        )
        wpimath.kinematics.SwerveDrive4Kinematics.desaturateWheelSpeeds(swerveStates, kMaxSpeed)  # type: ignore

        for i, module in enumerate(self.drivetrainC.modules):
            module_request = module.ModuleRequest().with_state(swerveStates[i])
            module.apply(module_request)

    def driveRobotRelative(self, speeds: wpimath.kinematics.ChassisSpeeds, feedforward: DriveFeedforwards):
        """
        Drive robot given chassis speeds and feedforward.

        :param speeds: `ChassisSpeeds` object with desired velocities
        :param feedforward: feedforward object required by PathPlanner
        """
        swerveStates = self.drivetrainC.kinematics.toSwerveModuleStates(speeds)
        for i, module in enumerate(self.drivetrainC.modules):
            module_request = module.ModuleRequest().with_state(swerveStates[i])
            module.apply(module_request)

    # ---------------------- Odometry ----------------------
    def updateOdometry(self) -> None:
        """Update the robot's odometry and publish telemetry."""
        self.odometry.update(
            self.getRotation2d(),
            tuple([module.get_position(True) for module in self.drivetrainC.modules])  # type: ignore
        )
        self.publishTelemetry()

    # ---------------------- Telemetry ----------------------
    def publishTelemetry(self):
        """Publish robot pose and module states to SmartDashboard."""
        pose = self.drivetrainC.get_state().pose
        self.pose_topic.set([pose.X(), pose.Y(), pose.rotation().degrees()])

        states = self.drivetrainC.get_state().module_states
        arr = []
        for s in states:
            arr += [s.speed, s.angle.radians()]
        self.module_states_topic.set(arr)

    # ---------------------- Utilities ----------------------
    def stopModules(self):
        """Stop all swerve modules using StaticBrake."""
        stop = StaticBrake()
        for modules in self.drivetrainC.modules:
            modules.drive_motor.set_control(stop)
            modules.steer_motor.set_control(stop)
            modules.encoder.set_control(stop)

    def getModuleStates(self):
        """
        Get the current state of each swerve module.

        :returns: tuple of 4 `SwerveModuleState` objects
        """
        return tuple([module.get_current_state() for module in self.drivetrainC.modules])
    
    def getModulePositions(self) -> tuple:
        """
        Get the current positions of each swerve module.

        :returns: tuple of 4 `SwerveModulePosition` objects
        """
        return tuple(module.get_position(True) for module in self.drivetrainC.modules)
    
    def setModuleStates(self, desiredStates: tuple[SwerveModuleState, SwerveModuleState, SwerveModuleState, SwerveModuleState]) -> None:
        """
        Set desired states for all swerve modules.

        :param desiredStates: tuple containing desired `SwerveModuleState` for each module
        """
        desiredStates = wpimath.kinematics.SwerveDrive4Kinematics.desaturateWheelSpeeds(
            desiredStates, kMaxSpeed
        )
        for i, module in enumerate(self.drivetrainC.modules):
            desiredStates[i].optimize(module.get_current_state().angle)
            module_request = module.ModuleRequest().with_state(desiredStates[i])
            module.apply(module_request)

    def getRobotRelativeSpeeds(self):
        """
        Get the robot-relative chassis speeds based on module states.

        :returns: `ChassisSpeeds` object
        """
        return self.drivetrainC.kinematics.toChassisSpeeds(self.getModuleStates())  # type: ignore


def get_module_constants(driveMotorId: int, turnMotorId: int, canCoderId: int):
    """
    Create and return a SwerveModuleConstants object.

    :param driveMotorId: CAN ID for the drive motor
    :param turnMotorId: CAN ID for the turn motor
    :param canCoderId: CAN ID for the CANCoder
    :returns: `SwerveModuleConstants` object with wheel and gear ratio parameters
    """
    driveConfig = TalonFXConfiguration()
    driveConfig.slot0.k_p = 2.5
    driveConfig.slot0.k_i = 0
    driveConfig.slot0.k_d = 0
    driveConfig.slot0.k_s = 6.4111
    driveConfig.slot0.k_v = 0.087032
    driveConfig.slot0.k_a = 0

    driveConfig.feedback.sensor_to_mechanism_ratio = kDriveGearRatio

    turnConfig = TalonFXConfiguration()
    turnConfig.slot0.k_p = 50
    turnConfig.slot0.k_i = 0
    turnConfig.slot0.k_d = 3.0889
    turnConfig.slot0.k_s = 0.21041
    turnConfig.slot0.k_v = 2.68
    turnConfig.slot0.k_a = 0.084645 

    turnConfig.feedback.sensor_to_mechanism_ratio = kTurnGearRatio

    return (SwerveModuleConstants()
                        .with_drive_motor_id(driveMotorId)
                        .with_steer_motor_id(turnMotorId)
                        .with_encoder_id(canCoderId)
                        .with_drive_motor_gear_ratio(kDriveGearRatio)
                        .with_steer_motor_gear_ratio(kTurnGearRatio)
                        .with_wheel_radius(kWheelRadius)
                        .with_drive_motor_initial_configs(driveConfig)
                        .with_steer_motor_initial_configs(turnConfig)
                        .with_encoder_initial_configs(CANcoderConfiguration()))
