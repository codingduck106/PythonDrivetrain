import wpilib
import wpimath
import wpimath.filter
import drivetrain
from ntcore import NetworkTableInstance
from robotcontainer import RobotContainer

class MyRobot(wpilib.TimedRobot):
    """2813 :D"""
    def robotInit(self):
        """Robot initialization"""
        self.container = RobotContainer(self.isSimulation())
        self.drive_controller = wpilib.PS4Controller(0)
        self.steer_controller = wpilib.PS4Controller(1)
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
        """runs periodically during any of the robot's cycles"""

        period = self.getPeriod()
        if self.isSimulation():
            self.swerve.update_pose_sim(period)
            pose = self.swerve.getPoseSim()
            speeds = self.swerve.getRobotRelativeSpeeds()
        else:
            self.swerve.updateOdometry()

            # Pose
            pose = self.swerve.getPose()

            # Chassis Speeds
            speeds = self.swerve.getRobotRelativeSpeeds()


        self.pose_pub.set([pose.X(), pose.Y(), pose.rotation().degrees()])
        self.speeds_pub.set([speeds.vx, speeds.vy, speeds.omega])

        # Mode
        mode_str = "Autonomous" if self.isAutonomous() else "Teleop" if self.isTeleop() else "Disabled"
        self.mode_pub.set(mode_str)

    def autonomousInit(self):
        """runs when auto begins"""
        self.autoCommand = self.container.getAutonomousCommand()
        if self.autoCommand:
            self.autoCommand.schedule()

    def teleopPeriodic(self):
        """runs during teleop"""
        self.driveWithJoystick(self.isSimulation(), True)
        
    def driveWithJoystick(self, sim: bool, fieldRelative: bool):
        """code to handle driving with the joystick"""
        xSpeed = -self.xLimiter.calculate(self.drive_controller.getLeftY()) * drivetrain.kMaxSpeed
        ySpeed = -self.yLimiter.calculate(self.drive_controller.getLeftX()) * drivetrain.kMaxSpeed
        rot = self.rotLimiter.calculate(self.steer_controller.getLeftX()) * drivetrain.kMaxAngularSpeed

        if sim:
            self.swerve.driveSim(xSpeed, ySpeed, rot / 45, self.getPeriod(), fieldRelative)
        else:
            self.swerve.drive(xSpeed, ySpeed, rot, fieldRelative, self.getPeriod())

    def disabledInit(self):
        """runs when robot is disabled"""
        self.container.drive.stopModules()

if __name__ == "__main__":
    wpilib.run(MyRobot)