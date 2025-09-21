from phoenix6.swerve import SimSwerveDrivetrain, SwerveDrivetrain, SwerveModule, SwerveModuleConstants
from phoenix6.utils import get_current_time_seconds
from wpimath.estimator import SwerveDrive4PoseEstimator
from wpimath.geometry import Pose2d, Rotation2d, Rotation3d
from ntcore import NetworkTable, StructPublisher
from wpilib import RobotController

class SimDrivetrain(SimSwerveDrivetrain):

    pose_estimator: SwerveDrive4PoseEstimator
    last_sim_update_seconds: float

    def __init__(self, network_table: NetworkTable, drivetrain: SwerveDrivetrain, module_constants: list[SwerveModuleConstants]):
        super().__init__(drivetrain.module_locations, drivetrain.pigeon2.sim_state, module_constants)
        self.drivetrain = drivetrain
        self.sim_pose = self.drivetrain.get_state().pose
        self.sim_current_pose = network_table.getStructTopic("simulated pose", Pose2d).publish()
        self.last_sim_update_seconds = 0
        self.pose_estimator = SwerveDrive4PoseEstimator(self.drivetrain.kinematics, # type: ignore
                                                        self.get_rotation3d().toRotation2d(),
                                                        tuple(self.get_module_positions(self.drivetrain)), # type: ignore
                                                        self.get_pose())
        gyroAngle = self.drivetrain.pigeon2.getRotation2d()
        module_positions = self.get_module_positions(self.drivetrain)


    def get_module_positions(self, drivetrain: SwerveDrivetrain):
        return list(map(SwerveModule.get_cached_position, drivetrain.modules))
    
    def reset_pose(self, pose: Pose2d):
        self.sim_pose = pose
        self.pose_estimator.resetPose(pose)

    def get_pose(self):
        return self.sim_pose
    
    def periodic(self):
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

    def get_rotation3d(self):
        return Rotation3d(self.sim_pose.rotation())

