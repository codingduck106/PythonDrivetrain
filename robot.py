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
        
        # if self.controller.getCircleButton():
        #     self.swerve.x_stance()
        # if self.controller.getL1Button():
        #     self.swerve.point_wheels(wpimath.geometry.Rotation2d(math.pi))
        # if self.controller.getCrossButton():
        #     self.swerve.stop()

        # self.driveWithJoystick()

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
        # self.is_driving_manually = False
        # self.autoCommand = self.container.getAutonomousCommand()
        # if self.autoCommand:
        #     self.autoCommand.schedule()
        if self.autonomous_command:
            self.autonomous_command.schedule()

    def teleopInit(self):
        """Runs when teleop begins - CRITICAL FIX"""
        if self.autonomous_command:
            self.autonomous_command.cancel()

    # def teleopPeriodic(self):
    #     """Runs during teleop - FIXED to prevent conflicts"""
    #     if self.controller.getCircleButton():
    #         self.swerve.point_wheels(wpimath.geometry.Rotation2d(self.controller.getLeftX()))
    #     if self.controller.getL2Button():
    #         self.swerve.stop()
        

    # def driveWithJoystick(self):
    #     """Fixed joystick driving with deadband and conflict prevention"""
        
    #     # Get raw joystick inputs
    #     raw_x = -self.controller.getLeftY()  # Forward/backward (inverted)
    #     raw_y = -self.controller.getLeftX()  # Left/right (inverted)
    #     raw_rot = -self.controller.getRightX()  # Rotation (inverted)
        
    #     # Apply deadband to prevent tiny movements causing oscillation
    #     DEADBAND = 0.15  # Increase this if still getting micro-movements
        
    #     x_speed = raw_x if abs(raw_x) > DEADBAND else 0.0
    #     y_speed = raw_y if abs(raw_y) > DEADBAND else 0.0
    #     rotation = raw_rot if abs(raw_rot) > DEADBAND else 0.0
        
    #     # Apply slew rate limiting
    #     x_speed = self.xLimiter.calculate(x_speed)
    #     y_speed = self.yLimiter.calculate(y_speed)
    #     rotation = self.rotLimiter.calculate(rotation)
        
    #     # Check if we're actually trying to move
    #     is_moving = abs(x_speed) > 0.01 or abs(y_speed) > 0.01 or abs(rotation) > 0.01
        
    #     if is_moving:
    #         # Only send drive command if we're actually moving
    #         self.swerve.drive(x_speed, y_speed, rotation, field_relative=True)
    #         self.last_drive_time = wpilib.Timer.getFPGATimestamp()
    #     else:
    #         # If not moving, only send stop command occasionally to avoid spam
    #         current_time = wpilib.Timer.getFPGATimestamp()
    #         if current_time - self.last_drive_time > 0.1:  # 100ms since last movement
    #             self.swerve.stop()
    #             self.last_drive_time = current_time

    def testExit(self):
        CommandScheduler.getInstance().cancelAll()