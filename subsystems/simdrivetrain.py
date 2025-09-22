from phoenix6.swerve import SimSwerveDrivetrain, SwerveDrivetrain, SwerveModule, SwerveModuleConstants, SwerveModulePosition
from phoenix6.utils import get_current_time_seconds
from wpimath.estimator import SwerveDrive4PoseEstimator
from wpimath.geometry import Pose2d, Rotation3d
from ntcore import NetworkTable
from wpilib import RobotController

class SimDrivetrain(SimSwerveDrivetrain):
    """A simulation drivetrain object."""

    pose_estimator: SwerveDrive4PoseEstimator
    last_sim_update_seconds: float

    def __init__(self, network_table: NetworkTable, drivetrain: SwerveDrivetrain, module_constants: list[SwerveModuleConstants]):
        """Initializes the sim drivetrain
        
        :param network_table: `NetworkTable` object used for logging the sim drivetrain's sim data.
        :param drivetrain: `SwerveDrivetrain` object, an instance of a `physical` drivetrain.
        :param module_constants: `list[SwerveModuleConstants], the module constants of the sim drivetrain."""
        super().__init__(drivetrain.module_locations, drivetrain.pigeon2.sim_state, module_constants) # calls the parent class's constructor, which is a phoenix6 simswerve.

        self.drivetrain = drivetrain
        self.sim_pose = self.drivetrain.get_state().pose
        self.sim_current_pose = network_table.getStructTopic("simulated pose", Pose2d).publish()
        self.last_sim_update_seconds = 0

        # creates a pose estimator for simulation.
        self.pose_estimator = SwerveDrive4PoseEstimator(self.drivetrain.kinematics, # type: ignore , we know it's a Swerve4Kinematics object or wtv it's called i forgor
                                                        self.get_rotation3d().toRotation2d(),
                                                        tuple(self.get_module_positions(self.drivetrain)), # type: ignore , we know there are exactly 4 swerve modules
                                                        self.get_pose())
        
        self.gyroAngle = self.drivetrain.pigeon2.getRotation2d()
        self.module_positions = self.get_module_positions(self.drivetrain)

    
    def reset_pose(self, pose: Pose2d):
        """Resets the simulation drivetrain's position
        
        :param pose: `Pose2d` object containing the pose to reset to."""
        self.sim_pose = pose
        self.pose_estimator.resetPose(pose)
    
    def periodic(self) -> None:
        """Stuff to do periodically during simulation.
        
        Updates the sim drivetrain's position, logs the current position to NetworkTables."""
        now = get_current_time_seconds()
        if self.last_sim_update_seconds == 0:
            self.last_sim_update_seconds = now
        self.update(
            now-self.last_sim_update_seconds,
            RobotController.getBatteryVoltage(),
            self.drivetrain.modules
        )
        self.last_sim_update_seconds = now

        gyroAngle = self.drivetrain.get_state().pose.rotation()
        wheel_positions = self.get_module_positions(self.drivetrain)

        self.pose_estimator.update(gyroAngle, tuple(wheel_positions)) # type: ignore
        self.sim_pose = self.pose_estimator.getEstimatedPosition()
        self.sim_current_pose.set(self.sim_pose)

    #=================================GETTERS==========================================

    def get_pose(self) -> Pose2d:
        """Returns the simulation pose.
        
        :returns: `Pose2d` object containing the current pose"""
        return self.sim_pose
    
    def get_module_positions(self, drivetrain: SwerveDrivetrain) -> list[SwerveModulePosition]:
        """Gets the swerve module positions on the given drivetrain
        
        :param drivetrain: `SwerveDrivetrain` object, an instance of the drivetrain.
        :returns: a list of `SwerveModulePosition` objects containing Swerve module positions, duh."""

        return list(map(SwerveModule.get_cached_position, drivetrain.modules)) # What does this do? basically, it applies a function to the list of modules, and then returns the output. think of it like a production line.

    def get_rotation3d(self) -> Rotation3d:
        """Gets the current 3d rotation of the simulation. 
        
        Since it's a simulation, the robot doesn't tip over unless we want it to, so the z value is always 0.
        
        :returns: `Rotation3d` object containing the sim bot's 3d rotation."""
        return Rotation3d(self.sim_pose.rotation())

