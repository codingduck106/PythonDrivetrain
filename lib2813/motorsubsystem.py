from phoenix6.hardware import TalonFX, CANcoder
from rev import SparkBase
from typing import Type
from phoenix6.units import rotation, radian
from phoenix6.signals import InvertedValue
from wpimath.controller import PIDController    

PIDMotor = Type("PIDMotor", (TalonFX, CANcoder), {})

class MotorSubsystemConfiguration:
    """Configuration for a motor subsystem."""

    def __init__(self, motor: TalonFX, encoder: CANcoder):
        self.motor = motor
        self.encoder = encoder
        self.controller = PIDController(0,0,0)
        self.acceptable_error = 0.5 # Default acceptable error
        self.start_pos = 0 # defualt start pos
        self.controlmode = SparkBase.ControlType.kDutyCycle
        self.rot_unit = rotation


    @classmethod
    def with_pidmotor(cls, motor: PIDMotor):
        return cls(motor, motor)