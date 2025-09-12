import wpilib
import wpimath.filter
from ntcore import NetworkTableInstance
from robotcontainer import RobotContainer

class MyRobot(wpilib.TimedRobot):
    """2813 :D"""

    def robotInit(self):
        """Robot initialization"""
        self.container = RobotContainer()
        self.controller = wpilib.PS4Controller(0)
        self.swerve = self.container.drive
        self.elevator = self.container.elevator  # Elevator reference

        # Slew rate limiters
        self.xLimiter = wpimath.filter.SlewRateLimiter(3)
        self.yLimiter = wpimath.filter.SlewRateLimiter(3)
        self.rotLimiter = wpimath.filter.SlewRateLimiter(3)

        # NetworkTables
        nt_instance = NetworkTableInstance.getDefault()
        self.nt_table = nt_instance.getTable("SmartDashboard")
        self.pose_pub = self.nt_table.getDoubleArrayTopic("RobotPose").publish()
        self.speeds_pub = self.nt_table.getDoubleArrayTopic("ChassisSpeeds").publish()
        self.mode_pub = self.nt_table.getStringTopic("RobotMode").publish()
        self.elevator_pos_pub = self.nt_table.getDoubleTopic("ElevatorPosition").publish()

        self.autoCommand = None

    def robotPeriodic(self):
        """runs periodically during any robot cycle"""
        self.swerve.updateOdometry()

        # Pose
        pose = self.swerve.getPose()
        self.pose_pub.set([pose.X(), pose.Y(), pose.rotation().degrees()])

        # Chassis speeds
        speeds = self.swerve.getRobotRelativeSpeeds()
        self.speeds_pub.set([speeds.vx, speeds.vy, speeds.omega])

        # Mode
        mode_str = "Autonomous" if self.isAutonomous() else "Teleop" if self.isTeleop() else "Disabled"
        self.mode_pub.set(mode_str)

        # Elevator
        self.elevator.periodic()
        self.elevator_pos_pub.set(self.elevator.get_position())

    def autonomousInit(self):
        """runs when auto begins"""
        self.autoCommand = self.container.getAutonomousCommand()
        if self.autoCommand:
            self.autoCommand.schedule()

    def teleopPeriodic(self):
        """runs during teleop"""
        self.driveWithJoystick(True)
        self.controlElevator()

    def driveWithJoystick(self, fieldRelative: bool):
        """code to handle driving with the joystick"""
        xSpeed = -self.xLimiter.calculate(self.controller.getLeftY()) * self.swerve.kMaxSpeed
        ySpeed = self.yLimiter.calculate(self.controller.getLeftX()) * self.swerve.kMaxSpeed
        rot = self.rotLimiter.calculate(self.controller.getRightX()) * self.swerve.kMaxAngularSpeed
        self.swerve.drive(xSpeed, ySpeed, rot, fieldRelative, self.getPeriod())

    def controlElevator(self):
        """Simple example: use controller buttons to move elevator"""
        if self.controller.getTriangleButton():
            self.elevator.set_position(self.elevator.Position.TOP)
        elif self.controller.getCrossButton():
            self.elevator.set_position(self.elevator.Position.BOTTOM)
        elif self.controller.getCircleButton():
            self.elevator.set_position(self.elevator.Position.TEST)

    def disabledInit(self):
        """runs when robot is disabled"""
        self.container.drive.stopModules()
