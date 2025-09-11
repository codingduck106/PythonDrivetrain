import wpilib
import wpimath
import wpimath.filter
import drivetrain
from ntcore import NetworkTableInstance
from robotcontainer import RobotContainer

class MyRobot(wpilib.TimedRobot):
    def robotInit(self):
        self.container = RobotContainer()
        self.controller = wpilib.PS4Controller(0)
        self.swerve = self.container.drive

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

        self.autoCommand = None

    def robotPeriodic(self):
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

    def autonomousInit(self):
        self.autoCommand = self.container.getAutonomousCommand()
        if self.autoCommand:
            self.autoCommand.schedule()

    def teleopPeriodic(self):
        self.driveWithJoystick(True)
        
    def driveWithJoystick(self, fieldRelative: bool):
        xSpeed = -self.xLimiter.calculate(self.controller.getLeftY()) * drivetrain.kMaxSpeed
        ySpeed = self.yLimiter.calculate(self.controller.getLeftX()) * drivetrain.kMaxSpeed
        rot = self.rotLimiter.calculate(self.controller.getRightX()) * drivetrain.kMaxAngularSpeed
        self.swerve.drive(xSpeed, ySpeed, rot, fieldRelative, self.getPeriod())

if __name__ == "__main__":
    wpilib.run(MyRobot)
