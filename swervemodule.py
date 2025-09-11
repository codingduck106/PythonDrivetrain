#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import math
import wpimath.controller
import wpimath.geometry
import wpimath.kinematics
from phoenix6.hardware import TalonFX, CANcoder
from phoenix6.canbus import CANBus
from phoenix6.configs import Slot0Configs, TalonFXConfiguration, CANcoderConfiguration
from phoenix6.controls import VelocityVoltage, PositionVoltage

kWheelRadius = 0.0508  # measured in meters
kDriveGearRatio = 6.75  # Motor rotations per wheel rotation
kTurnGearRatio = 12.8   # Motor rotations per module rotation
kModuleMaxAngularVelocity = math.pi
kModuleMaxAngularAcceleration = math.tau

class SwerveModule:
    def __init__(
        self,
        driveMotorId: int,
        turnMotorId: int,
        cancoderId: int,
        canBus: str | CANBus = ""
    ) -> None:
        """Constructs a SwerveModule with a drive motor, turn motor, and CANcoder
        :param driveMotorId: CAN ID for drive motor
        :param turnMotorId: CAN ID for turn motor
        :param cancoderId: CAN ID for CANcoder
        :param canBus: Optional: canbus name
        """

        # initialize motors and cancoder
        self.driveMotor = TalonFX(driveMotorId, canbus=canBus)
        self.turningMotor = TalonFX(turnMotorId, canbus=canBus)
        self.cancoder = CANcoder(cancoderId, canbus=canBus)

        # Configure drive motor
        driveConfig = TalonFXConfiguration()
        driveConfig.slot0.k_p = 2.5
        driveConfig.slot0.k_i = 0
        driveConfig.slot0.k_d = 0
        driveConfig.slot0.k_s = 6.4111
        driveConfig.slot0.k_v = 0.087032
        driveConfig.slot0.k_a = 0
        
        # Configure sensor-to-mechanism ratio for drive (gear ratio)
        driveConfig.feedback.sensor_to_mechanism_ratio = kDriveGearRatio
        
        # Apply the drive config to the motor
        self.driveMotor.configurator.apply(driveConfig)

        # Configure turn motor
        turnConfig = TalonFXConfiguration()
        turnConfig.slot0.k_p = 50
        turnConfig.slot0.k_i = 0
        turnConfig.slot0.k_d = 3.0889
        turnConfig.slot0.k_s = 0.21041
        turnConfig.slot0.k_v = 2.68
        turnConfig.slot0.k_a = 0.084645 
        
        # Configure sensor-to-mechanism ratio for turn (gear ratio)
        turnConfig.feedback.sensor_to_mechanism_ratio = kTurnGearRatio
        
        # Apply turn motor config to the motor
        self.turningMotor.configurator.apply(turnConfig)

        # Configure CANcoder
        cancoderConfig = CANcoderConfiguration()

        # Add any CANcoder-specific configuration here
        self.cancoder.configurator.apply(cancoderConfig)

        # Create control requests for Phoenix 6
        self.driveVelocityRequest = VelocityVoltage(0)
        self.turnPositionRequest = PositionVoltage(0)

        # WPILib PID controllers for higher-level control (if desired)
        self.drivePIDController = wpimath.controller.PIDController(0.1, 0, 0)
        self.turningPIDController = wpimath.controller.PIDController(1, 0, 0)
        
        # Feedforward controllers
        self.driveFeedforward = wpimath.controller.SimpleMotorFeedforwardMeters(1, 3)
        self.turnFeedforward = wpimath.controller.SimpleMotorFeedforwardMeters(1, 0.5)

        # Enable continuous input for turning controller
        self.turningPIDController.enableContinuousInput(-math.pi, math.pi)

    def getDriveDistanceMeters(self) -> float:
        """Get drive distance in meters
        
        :returns: drive distance in meters"""
        # Get position in rotations, convert to wheel rotations, then to distance
        motor_rotations = self.driveMotor.get_position().value
        wheel_rotations = motor_rotations / kDriveGearRatio
        distance_meters = wheel_rotations * (2 * math.pi * kWheelRadius)
        return distance_meters

    def getDriveVelocityMPS(self) -> float:
        """Get drive velocity in meters per second
        
        :returns: the drive velocity in meters per second"""
        # Get velocity in rotations per second, convert to wheel rotations, then to m/s
        motor_rps = self.driveMotor.get_velocity().value
        wheel_rps = motor_rps / kDriveGearRatio
        velocity_mps = wheel_rps * (2 * math.pi * kWheelRadius)
        return velocity_mps

    def getTurnAngleRadians(self) -> float:
        """Get turn angle in radians from CANcoder
        
        :returns: The turn angle in radians from CANcoder"""
        # Get absolute position from CANcoder in rotations, convert to radians
        cancoder_rotations = self.cancoder.get_absolute_position().value
        angle_radians = cancoder_rotations * 2 * math.pi
        return angle_radians

    def getState(self) -> wpimath.kinematics.SwerveModuleState:
        """Returns the current state of the module.

        :returns: The current state of the module.
        """
        return wpimath.kinematics.SwerveModuleState(
            self.getDriveVelocityMPS(),
            wpimath.geometry.Rotation2d(self.getTurnAngleRadians()),
        )

    def getPosition(self) -> wpimath.kinematics.SwerveModulePosition:
        """Returns the current position of the module.

        :returns: The current position of the module.
        """
        return wpimath.kinematics.SwerveModulePosition(
            self.getDriveDistanceMeters(),
            wpimath.geometry.Rotation2d(self.getTurnAngleRadians()),
        )

    def setDesiredState(
    self, desiredState: wpimath.kinematics.SwerveModuleState, useWPILibPID: bool = False
) -> None:
        """Sets the desired state for the module with optimization and telemetry."""

        current_angle = wpimath.geometry.Rotation2d(self.getTurnAngleRadians())

        # Optimize the reference state to avoid spinning further than 90 degrees
        desiredState.optimize(current_angle)

        # Scale speed by cosine of angle error
        desiredState.cosineScale(current_angle)

        # Apply the chosen control method
        if useWPILibPID:
            self.setDesiredStateWPILibPID(desiredState)
        else:
            self.setDesiredStatePhoenix6(desiredState)

        # Publish telemetry
        self.publishTelemetry(desiredState)


    def setDesiredStatePhoenix6(self, desiredState: wpimath.kinematics.SwerveModuleState):
        """Set desired state using Phoenix 6 built-in control
        
        :param desiredState: desired state with speed and angle"""
        # Convert desired velocity from m/s to rotations per second
        desired_velocity_mps = desiredState.speed
        desired_wheel_rps = desired_velocity_mps / (2 * math.pi * kWheelRadius)
        desired_motor_rps = desired_wheel_rps * kDriveGearRatio

        desired_angle_radians = desiredState.angle.radians()
        desired_angle_rotations = desired_angle_radians / (2 * math.pi)

        self.driveMotor.set_control(self.driveVelocityRequest.with_velocity(desired_motor_rps))
        self.turningMotor.set_control(self.turnPositionRequest.with_position(desired_angle_rotations))

        # Telemetry
        self.publishTelemetry(desiredState)


    def setDesiredStateWPILibPID(self, desiredState: wpimath.kinematics.SwerveModuleState):
        """Alternative: Set desired state using WPILib PID controllers
        
        :param desiredState: desired state with speed and angle"""
        # Calculate drive output using WPILib PID
        driveOutput = self.drivePIDController.calculate(
            self.getDriveVelocityMPS(), desiredState.speed
        )

        driveFeedforward = self.driveFeedforward.calculate(desiredState.speed)

        turnOutput = self.turningPIDController.calculate(self.getTurnAngleRadians(), desiredState.angle.radians())
        turnFeedforward = self.turnFeedforward.calculate(self.turningPIDController.getSetpoint())

        self.driveMotor.setVoltage(driveOutput + driveFeedforward)
        self.turningMotor.setVoltage(turnOutput + turnFeedforward)

        # Telemetry
        self.publishTelemetry(desiredState)

    def publishTelemetry(self, desiredState: wpimath.kinematics.SwerveModuleState | None = None):
        """Publishes the telemetry to NetworkTables
        
        :param desiredState: the desired swerve module state as a `SwerveModuleState`"""
        from ntcore import NetworkTableInstance
        nt = NetworkTableInstance.getDefault()
        table = nt.getTable("SwerveModule")

        # Create publishers once (ideally in __init__) and reuse
        drive_pub = table.getDoubleTopic("DriveVelocity_FL").publish()
        turn_pub = table.getDoubleTopic("TurnAngle_FL").publish()

        # Set values via the publisher
        drive_pub.set(self.getDriveVelocityMPS())
        turn_pub.set(self.getTurnAngleRadians())

        if desiredState:
            desired_drive_pub = table.getDoubleTopic("DesiredDrive_FL").publish()
            desired_angle_pub = table.getDoubleTopic("DesiredAngle_FL").publish()
            desired_drive_pub.set(desiredState.speed)
            desired_angle_pub.set(desiredState.angle.radians())
