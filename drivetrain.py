#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import math
import wpimath.geometry
import wpimath.kinematics
from phoenix6.hardware import Pigeon2
import swervemodule
from constants import *
from commands2 import Subsystem
import wpilib
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.controller import PPHolonomicDriveController
from pathplannerlib.config import RobotConfig, PIDConstants

kMaxSpeed = 3.0  # 3 meters per second
kMaxAngularSpeed = math.pi  # 1/2 rotation per second

class Drivetrain(Subsystem):
    """
    Represents a swerve drive style drivetrain.
    """
    
    def __init__(self) -> None:
        # Define module locations (distance from center of robot)
        # These values should match your robot's actual dimensions
        self.frontLeftLocation = wpimath.geometry.Translation2d(0.381, 0.381)
        self.frontRightLocation = wpimath.geometry.Translation2d(0.381, -0.381)
        self.backLeftLocation = wpimath.geometry.Translation2d(-0.381, 0.381)
        self.backRightLocation = wpimath.geometry.Translation2d(-0.381, -0.381)
        
        # Create swerve modules with updated constructor
        # SwerveModule(driveMotorId, turnMotorId, cancoderId, canBus="")
        # Update these CAN IDs to match your robot's configuration
        self.frontLeft = swervemodule.SwerveModule(FLConstants.DRIVE, FLConstants.TURN, FLConstants.CAN, "swerve")    # FL: drive=1, turn=2, cancoder=9
        self.frontRight = swervemodule.SwerveModule(FRConstants.DRIVE, FRConstants.TURN, FLConstants.CAN, "swerve")  # FR: drive=3, turn=4, cancoder=10
        self.backLeft = swervemodule.SwerveModule(BLConstants.DRIVE, BLConstants.TURN, BLConstants.CAN, "swerve")    # BL: drive=5, turn=6, cancoder=11
        self.backRight = swervemodule.SwerveModule(BRConstants.DRIVE, BRConstants.TURN, BRConstants.CAN, "swerve")   # BR: drive=7, turn=8, cancoder=12
        
        # Use Pigeon2 instead of AnalogGyro for better performance
        # Update CAN ID to match your robot's Pigeon2
        self.gyro = Pigeon2(GYRO, "swerve")  # CAN ID 13, default canbus
        
        # Create kinematics object
        self.kinematics = wpimath.kinematics.SwerveDrive4Kinematics(
            self.frontLeftLocation,
            self.frontRightLocation,
            self.backLeftLocation,
            self.backRightLocation,
        )
        
        # Create odometry object
        self.odometry = wpimath.kinematics.SwerveDrive4Odometry(
            self.kinematics,
            self.getRotation2d(),
            (
                self.frontLeft.getPosition(),
                self.frontRight.getPosition(),
                self.backLeft.getPosition(),
                self.backRight.getPosition(),
            ),
        )
        
        # Reset gyro
        self.gyro.reset()


        # Initialize pathplanner
        config = RobotConfig.fromGUISettings()
        AutoBuilder.configure(
            self.getPose,
            self.resetPose,
            self.getRobotRelativeSpeeds,
            lambda speeds, feedforwards: self.driveRobotRelative(speeds, feedforwards),
            PPHolonomicDriveController(
                PIDConstants(5.0, 0.0, 0.0),
                PIDConstants(5.0, 0.0, 0.0)
            ),
            config,
            self.shouldFlipPath,
            self,
        )
    
    def getRotation2d(self) -> wpimath.geometry.Rotation2d:
        """Get current rotation from gyro"""
        # Pigeon2 returns yaw in degrees, convert to Rotation2d
        yaw_degrees = self.gyro.get_yaw().value
        return wpimath.geometry.Rotation2d.fromDegrees(yaw_degrees)
    
    def getPose(self) -> wpimath.geometry.Pose2d:
        """Returns the current pose of the robot"""
        return self.odometry.getPose()
    
    def shouldFlipPath(self):
        return wpilib.DriverStation.getAlliance() == wpilib.DriverStation.Alliance.kRed
    def resetPose(self, pose: wpimath.geometry.Pose2d = wpimath.geometry.Pose2d()) -> None:
        """Resets the odometry to the specified pose"""
        self.odometry.resetPosition(
            self.getRotation2d(),
            (
                self.frontLeft.getPosition(),
                self.frontRight.getPosition(),
                self.backLeft.getPosition(),
                self.backRight.getPosition(),
            ),
            pose
        )
    
    def drive(
        self,
        xSpeed: float,
        ySpeed: float,
        rot: float,
        fieldRelative: bool,
        periodSeconds: float,
    ) -> None:
        """
        Method to drive the robot using joystick info.
        :param xSpeed: Speed of the robot in the x direction (forward).
        :param ySpeed: Speed of the robot in the y direction (sideways).
        :param rot: Angular rate of the robot.
        :param fieldRelative: Whether the provided x and y speeds are relative to the field.
        :param periodSeconds: Time period for discretization
        """
        # Create chassis speeds
        if fieldRelative:
            chassisSpeeds = wpimath.kinematics.ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeed, ySpeed, rot, self.getRotation2d()
            )
        else:
            chassisSpeeds = wpimath.kinematics.ChassisSpeeds(xSpeed, ySpeed, rot)
        
        # Discretize chassis speeds
        discretizedSpeeds = wpimath.kinematics.ChassisSpeeds.discretize(
            chassisSpeeds, periodSeconds
        )
        
        # Convert to swerve module states
        swerveModuleStates = self.kinematics.toSwerveModuleStates(discretizedSpeeds)
        
        # Desaturate wheel speeds
        wpimath.kinematics.SwerveDrive4Kinematics.desaturateWheelSpeeds(
            swerveModuleStates, kMaxSpeed
        )
        
        # Set desired states for each module
        self.frontLeft.setDesiredState(swerveModuleStates[0])
        self.frontRight.setDesiredState(swerveModuleStates[1])
        self.backLeft.setDesiredState(swerveModuleStates[2])
        self.backRight.setDesiredState(swerveModuleStates[3])

    def driveRobotRelative(self, speeds: wpimath.kinematics.ChassisSpeeds, feedForward):
        swerveModuleStates = self.kinematics.toSwerveModuleStates(speeds)
        self.frontLeft.setDesiredState(swerveModuleStates[0])
        self.frontRight.setDesiredState(swerveModuleStates[1])
        self.backLeft.setDesiredState(swerveModuleStates[2])
        self.backRight.setDesiredState(swerveModuleStates[3])
    
    def updateOdometry(self) -> None:
        """Updates the field relative position of the robot."""
        self.odometry.update(
            self.getRotation2d(),
            (
                self.frontLeft.getPosition(),
                self.frontRight.getPosition(),
                self.backLeft.getPosition(),
                self.backRight.getPosition(),
            ),
        )
    
    def stopModules(self) -> None:
        """Stop all swerve modules"""
        self.frontLeft.setDesiredState(wpimath.kinematics.SwerveModuleState(0, wpimath.geometry.Rotation2d(0)))
        self.frontRight.setDesiredState(wpimath.kinematics.SwerveModuleState(0, wpimath.geometry.Rotation2d(0)))
        self.backLeft.setDesiredState(wpimath.kinematics.SwerveModuleState(0, wpimath.geometry.Rotation2d(0)))
        self.backRight.setDesiredState(wpimath.kinematics.SwerveModuleState(0, wpimath.geometry.Rotation2d(0)))
    
    def getModuleStates(self) -> tuple:
        """Get current states of all modules"""
        return (
            self.frontLeft.getState(),
            self.frontRight.getState(),
            self.backLeft.getState(),
            self.backRight.getState(),
        )
    
    def getModulePositions(self) -> tuple:
        """Get current positions of all modules"""
        return (
            self.frontLeft.getPosition(),
            self.frontRight.getPosition(),
            self.backLeft.getPosition(),
            self.backRight.getPosition(),
        )
    
    def setModuleStates(self, desiredStates: tuple[wpimath.kinematics.SwerveModuleState, wpimath.kinematics.SwerveModuleState, wpimath.kinematics.SwerveModuleState, wpimath.kinematics.SwerveModuleState]) -> None:
        """Set desired states for all modules"""
        # Desaturate wheel speeds
        wpimath.kinematics.SwerveDrive4Kinematics.desaturateWheelSpeeds(
            desiredStates, kMaxSpeed
        )
        
        self.frontLeft.setDesiredState(desiredStates[0])
        self.frontRight.setDesiredState(desiredStates[1])
        self.backLeft.setDesiredState(desiredStates[2])
        self.backRight.setDesiredState(desiredStates[3])

    def getRobotRelativeSpeeds(self) -> wpimath.kinematics.ChassisSpeeds:
        return self.kinematics.toChassisSpeeds(self.getModuleStates())
        