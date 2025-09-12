from phoenix6.hardware import TalonFX
from phoenix6.signals import NeutralModeValue
from wpilib import RobotController

class Elevator:
    """Elevator subsystem (Pablo) with PID and voltage control."""

    TICKS_PER_ROTATION = 2048  # Falcon 500 integrated encoder

    class Position:
        BOTTOM = -0.212500
        TEST = 10
        TOP = 16.358496

    def __init__(self, motor1_id: int, motor2_id: int, invert_motor=False, invert_follower=False):
        # Master motor
        self.motor = TalonFX(motor1_id)
        self.motor.setNeutralMode(NeutralModeValue.BRAKE)
        self.invert_motor = invert_motor

        # Follower motor
        self.follower = TalonFX(motor2_id)
        self.invert_follower = invert_follower

        # PID constants
        self.kP = 0.2
        self.kI = 0.001
        self.kD = 0.001

        # PID state
        self.setpoint_rotations = 0.0
        self.integral = 0.0
        self.prev_error = 0.0
        self.acceptable_error_rotations = 1.7  # rotations

    def set_position(self, position_rotations: float):
        """Set target position in rotations."""
        self.setpoint_rotations = position_rotations

    def get_position(self) -> float:
        """Return current position in rotations."""
        ticks = self.motor.get_position().value_as_double
        return ticks / self.TICKS_PER_ROTATION

    def at_position(self) -> bool:
        """Check if elevator is within acceptable error of setpoint."""
        return abs(self.setpoint_rotations - self.get_position()) <= self.acceptable_error_rotations

    def get_applied_current(self) -> float:
        """Return motor supply current in amps."""
        return self.motor.get_supply_current().value_as_double

    def _update_motor_output(self, dt: float = 0.02):
        """Update motor output using PID and voltage clamping."""
        error = self.setpoint_rotations - self.get_position()
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        self.prev_error = error

        output_volts = self.kP * error + self.kI * self.integral + self.kD * derivative

        # Clamp output to ±6V
        output_volts = max(min(output_volts, 6), -6)

        # Handle inversion
        if self.invert_motor:
            output_volts = -output_volts

        # Convert volts to percent output for TalonFX
        percent_output = output_volts / RobotController.getBatteryVoltage()
        self.motor.set(percent_output)

    def periodic(self, dt: float = 0.02):
        """Call periodically to update motor output (default 20ms loop)."""
        self._update_motor_output(dt)
