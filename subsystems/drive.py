from phoenix6.hardware import TalonFX, CANcoder
from phoenix6.swerve import *
from pathplannerlib.auto import AutoBuilder, RobotConfig
from pathplannerlib.controller import PIDConstants, PPHolonomicDriveController
from phoenix6.units import *
from wpimath.units import inchesToMeters
from ntcore import NetworkTableInstance
from constants import *
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from commands2 import Subsystem

class SwerveDrive(Subsystem, SwerveDrivetrain):
    def __init__(self, ntInstance: NetworkTableInstance):
        swerveConstants = SwerveDrivetrainConstants().with_can_bus_name(GenericConstants.DRIVE_CAN_LOOP_NAME).with_pigeon2_id(GenericConstants.PIGEON_ID)

        factory = (SwerveModuleConstantsFactory()
                   .with_drive_motor_gear_ratio(DriveConstants.DRIVE_MOTOR_GEAR_RATIO)
                   .with_steer_motor_gear_ratio(DriveConstants.STEER_MOTOR_GEAR_RATIO)
                   .with_wheel_radius(inchesToMeters(DriveConstants.WHEEL_RADIUS))
                   .with_drive_motor_gains(DriveConstants.DRIVE_PID)
                   .with_steer_motor_gains(DriveConstants.STEER_PID)
                   .with_slip_current(DriveConstants.MOTOR_SLIP_CURRENT)
                   .with_drive_motor_closed_loop_output(
                       ClosedLoopOutputType.VOLTAGE
                   )
                   .with_steer_motor_closed_loop_output(
                       ClosedLoopOutputType.VOLTAGE
                   )
                   .with_feedback_source(
                       SteerFeedbackType.REMOTE_CANCODER
                   )
                   .with_coupling_gear_ratio(
                       DriveConstants.COUPLING_GEAR_RATIO
                   )
                   )
        
        fl, fr, bl, br = [factory.create_module_constants(
                        [GenericConstants.FRONT_LEFT_STEER_ID,
                         GenericConstants.FRONT_RIGHT_STEER_ID,
                         GenericConstants.BACK_LEFT_STEER_ID,
                         GenericConstants.BACK_RIGHT_STEER_ID][i],

                         [GenericConstants.FRONT_LEFT_DRIVE_ID,
                         GenericConstants.FRONT_RIGHT_DRIVE_ID,
                         GenericConstants.BACK_LEFT_DRIVE_ID,
                         GenericConstants.BACK_RIGHT_DRIVE_ID][i],

                         [GenericConstants.FRONT_LEFT_ENCODER_ID,
                         GenericConstants.FRONT_RIGHT_ENCODER_ID,
                         GenericConstants.BACK_LEFT_ENCODER_ID,
                         GenericConstants.BACK_RIGHT_ENCODER_ID][i],

                         [DriveConstants.FRONT_LEFT_ENCODER_OFFSET,
                          DriveConstants.FRONT_RIGHT_ENCODER_OFFSET,
                          DriveConstants.BACK_LEFT_ENCODER_OFFSET,
                          DriveConstants.BACK_RIGHT_ENCODER_OFFSET][i],

                          inchesToMeters([DriveConstants.DISTANCE_FROM_BOT_CENTER_TO_FRONT,
                           DriveConstants.DISTANCE_FROM_BOT_CENTER_TO_FRONT,
                           -DriveConstants.DISTANCE_FROM_BOT_CENTER_TO_FRONT,
                           -DriveConstants.DISTANCE_FROM_BOT_CENTER_TO_FRONT][i]),

                           inchesToMeters([DriveConstants.DISTANCE_FROM_BOT_CENTER_TO_LEFT,
                           -DriveConstants.DISTANCE_FROM_BOT_CENTER_TO_LEFT,
                           DriveConstants.DISTANCE_FROM_BOT_CENTER_TO_LEFT,
                           -DriveConstants.DISTANCE_FROM_BOT_CENTER_TO_LEFT][i]),

                           False,
                           True,
                           False
                        ) for i in range(4)]
        

        self.swerve = SwerveDrivetrain(
            TalonFX,
            TalonFX,
            CANcoder,
            swerveConstants,
            [fl, fr, bl, br]
        )

        table = ntInstance.getTable("Drive")
        self.pose_pub = table.getStructTopic("Current Drive Pose", Pose2d).publish()

        self.cur_drive_state_pub = table.getStructArrayTopic("Current Drive State", SwerveModuleState).publish()

        self.expected_drive_state_pub = table.getStructArrayTopic("Expected Drive State", SwerveModuleState).publish()

        