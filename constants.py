from commands2.button import CommandPS4Controller
from phoenix6.units import volt, rotation, degree, inch, meters_per_second, rotations_per_second, ampere
from phoenix6.configs import Slot0Configs
from pathplannerlib.config import PIDConstants


class GenericConstants:
    # WARNING: The mechanisms and the drivetrain are on separate CAN loops, but DO NOT reuse CAN IDs.
    # Can ID reuse will mess with simulation.

    # Drivetrain Constants

    # CAN Loop identifier for swerve drive devices (motors and encoders).
    DRIVE_CAN_LOOP_NAME = "drive"

    # Drivetrain CAN IDs

    # CAN ID for front right steer motor.
    FRONT_RIGHT_STEER_ID = 1

    # CAN ID for front right encoder.
    FRONT_RIGHT_ENCODER_ID = 2

    # CAN ID for front right drive motor.
    FRONT_RIGHT_DRIVE_ID = 3

    # CAN ID for back right steer motor.
    BACK_RIGHT_STEER_ID = 4

    # CAN ID for back right encoder.
    BACK_RIGHT_ENCODER_ID = 5

    # CAN ID for back right drive motor.
    BACK_RIGHT_DRIVE_ID = 6

    # CAN ID for back left steer motor.
    BACK_LEFT_STEER_ID = 7

    # CAN ID for back left encoder.
    BACK_LEFT_ENCODER_ID = 8

    # CAN ID for back left drive motor.
    BACK_LEFT_DRIVE_ID = 9

    # CAN ID for front left steer motor.
    FRONT_LEFT_STEER_ID = 10

    # CAN ID for front left encoder.
    FRONT_LEFT_ENCODER_ID = 11

    # CAN ID for front left drive motor.
    FRONT_LEFT_DRIVE_ID = 12

    # Inertial Measurement Unit (IMU) CAN ID.
    PIGEON_ID = 13

    # Mechanism Constants.

    # CAN Loop identifier for non-drive devices.
    MECHANISM_CAN_LOOP_NAME = "rio"  # TODO: See if we can change this to "mech"

    # Ground Intake Can IDs

    # CAN ID for ground intake pivot motor.
    GROUND_INTAKE_PIVOT_ID = 21

    # CAN ID for ground intake wheel motor.
    GROUND_INTAKE_WHEEL_ID = 22

    # Elevator and Arm Can IDs

    # CAN ID for the elevator motor
    ELEVATOR_MOTOR_ID = 23

    # CAN ID for the arm pivot encoder.
    ARM_PIVOT_ENCODER_ID = 24

    # CAN ID for the arm pivot motor.
    ARM_PIVOT_MOTOR_ID = 25

    # CAN ID for the arm grabber (intake) motor.
    ARM_WHEEL_MOTOR_ID = 26

    # CAN ID for the CAN range sensor used by the arm wheels.
    CAN_RANGE_ID = 27

    class DriverConstants:
        # Pre-set driver controller instance.
        DRIVER_CONTROLLER = CommandPS4Controller(0)

    class OperatorConstants:
    # Pre-set operator controller instance.
        OPERATOR_CONTROLLER = CommandPS4Controller(1)


    # Triggers for elevator and arm positions
    CORAL_GROUND_TRIGGER = (
        DriverConstants.DRIVER_CONTROLLER.L2()
        or OperatorConstants.OPERATOR_CONTROLLER.L2()
    )  # also used by ground intake
    OUTTAKE_TRIGGER = (
        DriverConstants.DRIVER_CONTROLLER.R2() or OperatorConstants.OPERATOR_CONTROLLER.R2()
    )
    CORAL_L4_TRIGGER = OperatorConstants.OPERATOR_CONTROLLER.triangle()
    CORAL_L3_TRIGGER = OperatorConstants.OPERATOR_CONTROLLER.square()
    CORAL_L2_TRIGGER = OperatorConstants.OPERATOR_CONTROLLER.circle()
    CORAL_L1_TRIGGER = OperatorConstants.OPERATOR_CONTROLLER.cross()
    BARGE_TRIGGER = OperatorConstants.OPERATOR_CONTROLLER.povUp()
    ALGAE_L3_TRIGGER = OperatorConstants.OPERATOR_CONTROLLER.povLeft()
    ALGAE_L2_TRIGGER = OperatorConstants.OPERATOR_CONTROLLER.povRight()
    ALGAE_GROUND_TRIGGER = OperatorConstants.OPERATOR_CONTROLLER.povDown()
    WILD_CARD = OperatorConstants.OPERATOR_CONTROLLER.PS()

    ALGAE_LOLLIPOP_TRIGGER = OperatorConstants.OPERATOR_CONTROLLER.share()

class ArmConstants:
    ARM_INTAKE_VOLTAGE = volt(5)
    ARM_OUTTAKE_VOLTAGE = volt(-3)

    ACCEPTABLE_ERROR_DEGREES = degree(1.0)

    # PID Constants
    ARM_PIVOT_P = 61
    ARM_PIVOT_I = 2
    ARM_PIVOT_D = 0

class ElevatorConstants:

    # PID Constants
    ELEVATOR_P = 1.08
    ELEVATOR_I = 0.02
    ELEVATOR_D = 0.13

    ACCEPTABLE_ERROR_ROTATIONS = rotation(0.1)

class GroundIntakeConstants:
    PIVOT_UP_POSITION_ROTATIONS = rotation(-0.25)
    PIVOT_DOWN_POSITION_ROTATIONS = rotation(0.25)

    ACCEPTABLE_ERROR = 0.1

    PIVOT_P = 1
    PIVOT_I = 0
    PIVOT_D = 0

    INTAKE_VOLTAGE = volt(9)

    OUTTAKE_VOLTAGE = volt(-9)

class DriveConstants:
    DISTANCE_FROM_BOT_CENTER_TO_FRONT = inch(11.2375)

    DISTANCE_FROM_BOT_CENTER_TO_LEFT = inch(11.2375)

    FRONT_LEFT_ENCODER_OFFSET = rotation(0.04492)

    FRONT_RIGHT_ENCODER_OFFSET = rotation(0.470215)

    BACK_LEFT_ENCODER_OFFSET = rotation(0.478516)

    BACK_RIGHT_ENCODER_OFFSET = rotation(0.497314)

    DRIVE_MOTOR_GEAR_RATIO = 6.75

    STEER_MOTOR_GEAR_RATIO = 150/7

    WHEEL_RADIUS = inch(3.5/2)

    MAX_LINEAR_VELOCITY = meters_per_second(4)

    MAX_ANGULAR_VELOCITY = rotations_per_second(1.5)

    MOTOR_SLIP_CURRENT = ampere(90)

    COUPLING_GEAR_RATIO = 3.5

    DRIVE_PID = (Slot0Configs()
                 .with_k_p(0.1)
                 .with_k_i(0)
                 .with_k_d(0)
                 .with_k_s(0)
                 .with_k_v(0.124)
                 .with_k_a(0))
    
    STEER_PID = (Slot0Configs()
                 .with_k_p(100)
                 .with_k_i(0)
                 .with_k_d(0.5)
                 .with_k_s(0.1)
                 .with_k_v(2.66)
                 .with_k_a(0))


    TRANSLATION_AUTO_PID = PIDConstants(5,0,0)

    ROTATION_AUTO_PID = PIDConstants(5,0,0)