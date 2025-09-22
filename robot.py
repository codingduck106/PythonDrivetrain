import wpilib
import wpimath.filter
from ntcore import NetworkTableInstance
from robotcontainer import RobotContainer
from wpilib import DriverStation
from commands2 import CommandScheduler

class MyRobot(wpilib.TimedRobot):
    """Robot class. Notice it has no constructor. 
    
    I don't know why I felt the need to point that out. Oh well"""
    
    def robotInit(self) -> None:
        """Robot initialization"""

        self.alliance = DriverStation.getAlliance() # the alliance we are on
        self.container = RobotContainer() # ROBOT CONTAINER RAHHH
        self.controller = wpilib.PS4Controller(0) # controller we use.
        self.swerve = self.container.drive # drivetrain!
        
        
        # Logging stuff
        nt_instance = NetworkTableInstance.getDefault()
        self.nt_table = nt_instance.getTable("SmartDashboard")
        self.pose_pub = self.nt_table.getDoubleArrayTopic("RobotPose").publish()
        self.speeds_pub = self.nt_table.getDoubleArrayTopic("ChassisSpeeds").publish()
        self.mode_pub = self.nt_table.getStringTopic("RobotMode").publish()
        self.states_pub = self.nt_table.getDoubleArrayTopic("SwerveModuleStates").publish()
        

        self.autonomous_command = None
        

    def robotPeriodic(self) -> None:
        """Runs periodically."""

        CommandScheduler.getInstance().run() # ok, so what this does is, uhhh it gets the scheduled commands and then runs them. I think. I'm not fully sure how it works. Command based programming is extremely wonky.
        self.updateTelemetry()

    def updateTelemetry(self) -> None:
        """Update telemetry"""
        try:
            # gets the drivetrain's data
            pose = self.swerve.get_pose()
            speeds = self.swerve.get_robot_relative_speeds()
            module_states = self.swerve.drivetrain.get_state().module_states
            
            # publishes drivetrain data
            self.pose_pub.set([pose.X(), pose.Y(), pose.rotation().degrees()])
            self.speeds_pub.set([speeds.vx, speeds.vy, speeds.omega])
            self.states_pub.set([state.angle.degrees() for state in module_states])
            
            # publishes the current mode.
            mode_str = "Autonomous" if self.isAutonomous() else "Teleop" if self.isTeleop() else "Disabled"
            self.mode_pub.set(mode_str)
        except Exception as e:
            print(f"Telemetry error: {e}")

    def autonomousInit(self):
        """Runs when auto begins. Basically just runs the Autonomous commands as seen here."""
        self.autonomous_command = self.container.getAutonomousCommand()
        if self.autonomous_command:
            self.autonomous_command.schedule()


    def teleopInit(self):
        """Runs when teleop begins"""
        if self.autonomous_command:
            self.autonomous_command.cancel()


    def testExit(self):
        """Cancels all commands when the robot exits testing mode."""
        CommandScheduler.getInstance().cancelAll()