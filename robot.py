import wpilib
import wpimath.filter
# from drivetrain import Drivetrain
from ntcore import NetworkTableInstance
from robotcontainer import RobotContainer
from wpilib import DriverStation
from phoenix6.swerve import SwerveModuleState
import math
from commands2 import CommandScheduler

class MyRobot(wpilib.TimedRobot):
    """Fixed robot class to prevent control conflicts"""
    
    def robotInit(self):
        """Robot initialization"""
        self.alliance = DriverStation.getAlliance()
        self.container = RobotContainer(self.alliance)
        self.controller = wpilib.PS4Controller(0)
        self.swerve = self.container.drive
        
        # Slew rate limiters
        self.xLimiter = wpimath.filter.SlewRateLimiter(3)
        self.yLimiter = wpimath.filter.SlewRateLimiter(3)
        self.rotLimiter = wpimath.filter.SlewRateLimiter(3)
        
        # Control state tracking
        self.last_drive_time = 0
        self.is_driving_manually = False
        
        # NetworkTables (reduced frequency updates)
        nt_instance = NetworkTableInstance.getDefault()
        self.nt_table = nt_instance.getTable("SmartDashboard")
        self.pose_pub = self.nt_table.getDoubleArrayTopic("RobotPose").publish()
        self.speeds_pub = self.nt_table.getDoubleArrayTopic("ChassisSpeeds").publish()
        self.mode_pub = self.nt_table.getStringTopic("RobotMode").publish()
        self.states_pub = self.nt_table.getDoubleArrayTopic("SwerveModuleStates").publish()
        
        # Telemetry counter to reduce update frequency
        self.telemetry_counter = 0

        self.autonomous_command = None
        

    def robotPeriodic(self):
        """Runs periodically - REDUCED telemetry frequency"""

        CommandScheduler.getInstance().run()
        # Only update telemetry every 5th cycle (100ms instead of 20ms)
        self.telemetry_counter += 1
        if self.telemetry_counter >= 5:
            self.telemetry_counter = 0
            self.updateTelemetry()

    def updateTelemetry(self):
        """Update telemetry at reduced frequency"""
        try:
            pose = self.swerve.get_pose()
            speeds = self.swerve.get_robot_relative_speeds()
            module_states = self.swerve.drivetrain.get_state().module_states
            
            self.pose_pub.set([pose.X(), pose.Y(), pose.rotation().degrees()])
            self.speeds_pub.set([speeds.vx, speeds.vy, speeds.omega])
            self.states_pub.set([state.angle.degrees() for state in module_states])
            
            mode_str = "Autonomous" if self.isAutonomous() else "Teleop" if self.isTeleop() else "Disabled"
            self.mode_pub.set(mode_str)
        except Exception as e:
            print(f"Telemetry error: {e}")

    def autonomousInit(self):
        """Runs when auto begins"""
        self.autonomous_command = self.container.getAutonomousCommand()
        if self.autonomous_command:
            self.autonomous_command.schedule()

    def teleopInit(self):
        """Runs when teleop begins - CRITICAL FIX"""
        if self.autonomous_command:
            self.autonomous_command.cancel()


    def testExit(self):
        CommandScheduler.getInstance().cancelAll()