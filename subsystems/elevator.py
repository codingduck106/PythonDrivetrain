from commands2 import Subsystem
from phoenix6.units import inch
from wpimath.units import inchesToMeters
from ntcore import NetworkTableInstance
from constants import GenericConstants, ElevatorConstants
from phoenix6.configs import Slot0Configs, FeedbackConfigs
from phoenix6.controls import PositionDutyCycle, VoltageOut
from phoenix6.hardware import TalonFX

from enum import Enum


def get_pos_value(name: str) -> float:
    return Elevator.POSITIONS[name].value / ElevatorConstants.HEIGHT_PER_ROTATION_INCHES

class Elevator(Subsystem):
    POSITIONS = Enum('POSITIONS', {
        "READY_HANDOFF_S1": inch(16.221),
        "STOW_S2": inch(0.00),
        "HANDOFF_C0": inch(12.5),
        "CORAL_1_C1": inch(14.5),
        "CORAL_2_C2": inch(14.5),
        "CORAL_2_OUT_C2_1": inch(14.5),
        "CORAL_3_C3": inch(25.0),
        "CORAL_3_OUT_C3_1": inch(25.0),
        "CORAL_4_C4": inch(53.0),
        "CORAL_4_OUT_C4_1": inch(53.0),
        "ARM_SWING_HEIGHT_I2": inch(14.5),
        "ALGAE_GROUND_A0": inch(0.00),
        "ALGAE_LOLLIPOP_A1": inch(2.00),
        "ALGAE_LOW_A2": inch(21.5),
        "ALGAE_HIGH_A3": inch(38.0),
        "BARGE_A4": inch(56.0)
    })
    
    def __init__(self, ntInstance: NetworkTableInstance):
        super().__init__()

        self.motor = TalonFX(GenericConstants.ELEVATOR_MOTOR_ID)

        self.motor.configurator.apply(
            Slot0Configs()
            .with_k_p(ElevatorConstants.ELEVATOR_P)
            .with_k_i(ElevatorConstants.ELEVATOR_I)
            .with_k_d(ElevatorConstants.ELEVATOR_D),
        )

        self.motor.configurator.apply(
            FeedbackConfigs()
            .with_sensor_to_mechanism_ratio(78)
        )

        self.currentSetpoint = get_pos_value("STOW_S2")

        # Publishers

        table = ntInstance.getTable("Elevator")
        self.setpoint_pub = table.getDoubleTopic("Setpoint (Inches)").publish()
        self.position_pub = table.getDoubleTopic("Position (Inches)").publish()
        self.at_position_pub = table.getBooleanTopic("At Position").publish()

    def periodic(self) -> None:
        self.setpoint_pub.set(self.currentSetpoint * ElevatorConstants.HEIGHT_PER_ROTATION_INCHES)
        self.position_pub.set(self.motor.get_position().value * ElevatorConstants.HEIGHT_PER_ROTATION_INCHES)
        at_position = abs(self.motor.get_position().value - self.currentSetpoint) < ElevatorConstants.ACCEPTABLE_ERROR_ROTATIONS
        self.at_position_pub.set(at_position)

    def set_position(self, position_name: str) -> None:
        self.currentSetpoint = get_pos_value(position_name)
        self.motor.set_control(PositionDutyCycle(get_pos_value(position_name)))

    def set_voltage(self, voltage: float) -> None:
        self.motor.set_control(VoltageOut(voltage))