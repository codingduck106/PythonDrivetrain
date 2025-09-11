import math
import wpimath.geometry
from wpimath.geometry import Translation2d
import wpimath.kinematics
from wpimath.kinematics import SwerveModuleState
from phoenix6.hardware import Pigeon2
from swervemodule import SwerveModule
from constants import *
from commands2 import Subsystem
import wpilib
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.controller import PPHolonomicDriveController
from pathplannerlib.config import RobotConfig, PIDConstants
from ntcore import NetworkTableInstance

kMaxSpeed = 3.0  # meters/sec
kMaxAngularSpeed = math.pi  # rad/sec

class Drivetrain(Subsystem):
    def __init__(self) -> None:
        """Initializes a drivetrain subsystem"""


        # Module positions
        self.frontLeftLocation = wpimath.geometry.Translation2d(0.381, 0.381)
        self.frontRightLocation = wpimath.geometry.Translation2d(0.381, -0.381)
        self.backLeftLocation = wpimath.geometry.Translation2d(-0.381, 0.381)
        self.backRightLocation = wpimath.geometry.Translation2d(-0.381, -0.381)
        
        # Create swerve modules with updated constructor
        # SwerveModule(driveMotorId, turnMotorId, cancoderId, canBus="")
        # Update these CAN IDs to match your robot's configuration
        self.frontLeft = SwerveModule(FLConstants.DRIVE, FLConstants.TURN, FLConstants.CAN, "swerve") 
        self.frontRight = SwerveModule(FRConstants.DRIVE, FRConstants.TURN, FLConstants.CAN, "swerve")
        self.backLeft = SwerveModule(BLConstants.DRIVE, BLConstants.TURN, BLConstants.CAN, "swerve") 
        self.backRight = SwerveModule(BRConstants.DRIVE, BRConstants.TURN, BRConstants.CAN, "swerve") 
        
        # Use Pigeon2 instead of AnalogGyro for better performance
        self.gyro = Pigeon2(GYRO, "swerve")
        
        # Create kinematics object

        self.kinematics = wpimath.kinematics.SwerveDrive4Kinematics(
            self.frontLeftLocation,
            self.frontRightLocation,
            self.backLeftLocation,
            self.backRightLocation,
        )
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

        # NetworkTables
        nt_instance = NetworkTableInstance.getDefault()
        self.nt_table = nt_instance.getTable("SmartDashboard")
        self.pose_topic = self.nt_table.getDoubleArrayTopic("DrivetrainPose").publish()
        self.module_states_topic = self.nt_table.getDoubleArrayTopic("DrivetrainModuleStates").publish()

        # AutoBuilder
        config = RobotConfig.fromGUISettings()
        AutoBuilder.configure(
            self.getPose,
            self.resetPose,
            self.getRobotRelativeSpeeds,
            lambda speeds, ff: self.driveRobotRelative(speeds, ff),
            PPHolonomicDriveController(
                PIDConstants(5.0, 0.0, 0.0),
                PIDConstants(5.0, 0.0, 0.0),
            ),
            config,
            self.shouldFlipPath,
            self,
        )

    # ---------------------- Basic Methods ----------------------
    def getRotation2d(self) -> wpimath.geometry.Rotation2d:
        """Get current rotation from gyro
        
        :returns: current rotation from gyro as a Rotation2d object"""
        yaw_deg = self.gyro.get_yaw().value
        return wpimath.geometry.Rotation2d.fromDegrees(yaw_deg)

    def getPose(self) -> wpimath.geometry.Pose2d:
        """Returns the current Pose as a Pose2d
        
        :returns: a `Pose2d` object representing the current pose"""
        return self.odometry.getPose()

    def shouldFlipPath(self):
        """returns whether the autobuilder should flip the path or not
        
        :returns: `bool` """
        return wpilib.DriverStation.getAlliance() == wpilib.DriverStation.Alliance.kRed

    def resetPose(self, pose: wpimath.geometry.Pose2d | None = None) -> None:
        """Resets the odometry to the specified pose
        
        :param pose: a specified pose as a Pose2d object"""
      
        if pose is None:
            pose = wpimath.geometry.Pose2d()
        self.odometry.resetPosition(
            self.getRotation2d(),
            (
                self.frontLeft.getPosition(),
                self.frontRight.getPosition(),
                self.backLeft.getPosition(),
                self.backRight.getPosition(),
            ),
            pose,
        )

    # ---------------------- Driving ----------------------
    def drive(self, xSpeed, ySpeed, rot, fieldRelative: bool, periodSeconds: float):
        """Drives the robot based on speed, and rotation
        
        :param xSpeed: the current x speed of the robot
        :param ySpeed: the current y speed of the robot
        :param fieldRelative: whether or not the given values are field relative or not
        :param periodSeconds: period of time for discretization of ChassisSpeeds"""

        if fieldRelative:
            chassisSpeeds = wpimath.kinematics.ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeed, ySpeed, rot, self.getRotation2d()
            )
        else:
            chassisSpeeds = wpimath.kinematics.ChassisSpeeds(xSpeed, ySpeed, rot)

        # Discretization of speeds  
        discretized = wpimath.kinematics.ChassisSpeeds.discretize(chassisSpeeds, periodSeconds)
        swerveStates = self.kinematics.toSwerveModuleStates(discretized)
        wpimath.kinematics.SwerveDrive4Kinematics.desaturateWheelSpeeds(swerveStates, kMaxSpeed)

        # initialize swerve modules
        self.frontLeft.setDesiredState(swerveStates[0])
        self.frontRight.setDesiredState(swerveStates[1])
        self.backLeft.setDesiredState(swerveStates[2])
        self.backRight.setDesiredState(swerveStates[3])

    def driveRobotRelative(self, speeds, feedforward):
        """Drives robot given a `ChassisSpeeds` object, and a feedforward object.
        
        :param speeds: a `ChassisSpeeds` object, which are the given speeds
        :param feedforward: the feedforward for the drive. this drivetrain doesn't need one, but pathplanner requires that it be there."""
        swerveStates = self.kinematics.toSwerveModuleStates(speeds)
        self.frontLeft.setDesiredState(swerveStates[0])
        self.frontRight.setDesiredState(swerveStates[1])
        self.backLeft.setDesiredState(swerveStates[2])
        self.backRight.setDesiredState(swerveStates[3])

    # ---------------------- Odometry ----------------------
    def updateOdometry(self) -> None:
        """Updates the robot's field odometry"""
        self.odometry.update(
            self.getRotation2d(),
            (
                self.frontLeft.getPosition(),
                self.frontRight.getPosition(),
                self.backLeft.getPosition(),
                self.backRight.getPosition(),
            ),
        )
        self.publishTelemetry()

    # ---------------------- Telemetry ----------------------
    def publishTelemetry(self):
        """Publishes the robot's telemetry to SmartDashboard"""
        # Pose
        pose = self.getPose()
        self.pose_topic.set([pose.X(), pose.Y(), pose.rotation().degrees()])

        # Module states
        states = self.getModuleStates()
        arr = []
        for s in states:
            arr += [s.speed, s.angle.radians()]
        self.module_states_topic.set(arr)

    # ---------------------- Utilities ----------------------
    def stopModules(self):
        """Stops all swerve modules"""
        zeroState = wpimath.kinematics.SwerveModuleState(0, wpimath.geometry.Rotation2d(0))
        self.frontLeft.setDesiredState(zeroState)
        self.frontRight.setDesiredState(zeroState)
        self.backLeft.setDesiredState(zeroState)
        self.backRight.setDesiredState(zeroState)

    def getModuleStates(self):
        """Get current states of all modules
        Order: FL, FR, BL, BR

        :returns: a tuple containing 4 SwerveModuleState objects, representing the state of each module"""
        return (
            self.frontLeft.getState(),
            self.frontRight.getState(),
            self.backLeft.getState(),
            self.backRight.getState(),
        )
    
    def getModulePositions(self) -> tuple:
        """Get current positions of all modules
        Order: FL, FR, BL, BR

        :returns: a tuple containing 4 SwerveModulePosition objects, representing the position of each module."""
        
        return (
            self.frontLeft.getPosition(),
            self.frontRight.getPosition(),
            self.backLeft.getPosition(),
            self.backRight.getPosition(),
        )
    
    def setModuleStates(self, desiredStates: tuple[SwerveModuleState, SwerveModuleState, SwerveModuleState, SwerveModuleState]) -> None:
        """Set desired states for all modules
        Order: FL, FR, BL, BR
        
        :param desiredStates: a tuple containing SwerveModuleState objects containing the desired state for each module."""
        # Desaturate wheel speeds
        wpimath.kinematics.SwerveDrive4Kinematics.desaturateWheelSpeeds(
            desiredStates, kMaxSpeed
        )
        
        self.frontLeft.setDesiredState(desiredStates[0])
        self.frontRight.setDesiredState(desiredStates[1])
        self.backLeft.setDesiredState(desiredStates[2])
        self.backRight.setDesiredState(desiredStates[3])

    def getRobotRelativeSpeeds(self):
        """Returns a ChassisSpeeds object representing the robot relative speeds
        
        :returns: a `ChassisSpeeds` object"""
        return self.kinematics.toChassisSpeeds(self.getModuleStates())
