from phoenix6.signals import InvertedValue
from phoenix6.hardware import TalonFX
from phoenix6.configs import CurrentLimitsConfigs, TalonFXConfiguration, SlotConfigs
from phoenix6.controls import VelocityDutyCycle, MotionMagicDutyCycle, VoltageOut, DutyCycleOut, StrictFollower, Follower
from phoenix6.units import rotation, radian
from phoenix6.signals import NeutralModeValue
from wpimath.units import radiansToRotations
from rev import SparkBase

class TalonFXWrapper:
    followers = []
    
    def __init__(self, id: int, canbus: str, invertType: InvertedValue):
        self.motor = TalonFX(id, canbus)
        config = TalonFXConfiguration()
        config.motor_output.inverted = invertType
        config.current_limits = CurrentLimitsConfigs().with_stator_current_limit(40).with_supply_current_limit_enable(True)
        self.motor.configurator.apply(config)

    @classmethod
    def from_id_and_invert(cls, id: int, invertType: InvertedValue):
        return cls(id, "", invertType)
    
    def set(self, control_mode: SparkBase.ControlType, demand: float, ff: float):
        match control_mode:
            case SparkBase.ControlType.kVelocity:
                v = VelocityDutyCycle(demand).with_feed_forward(ff)
                self.motor.set_control(v)
            case SparkBase.ControlType.kPosition:
                mm = MotionMagicDutyCycle(demand).with_feed_forward(ff)
                self.motor.set_control(mm)
            case SparkBase.ControlType.kVoltage:
                vo = VoltageOut(demand)
                self.motor.set_control(vo)
            case _: # default case
                dc = DutyCycleOut(demand)
                self.motor.set_control(dc)
    
    def position(self) -> rotation:
        return rotation(self.motor.get_position().value_as_double)
    
    def get_applied_current(self):
        return self.motor.get_stator_current().value

    def set_position(self, position: radian):
        self.motor.set_position(radiansToRotations(position))
    
    def get_velocity(self):
        return self.motor.get_velocity().value
    
    def setNeutralMode(self, mode: NeutralModeValue):
        self.motor.setNeutralMode(mode)

    def configPIDF(self, slot: int, p: float, i: float, d: float, f: float):
        conf = SlotConfigs()
        conf.slot_number = slot
        self.motor.configurator.apply(conf.with_k_p(p)
                                      .with_k_i(i)
                                      .with_k_d(d)
                                      .with_k_v(f))
    def from_slot0(self, p: float, i: float, d: float, f: float):
        return self.configPIDF(0, p, i, d, f)
    
    def configPID(self, slot: int, p: float, i: float, d: float):
        return self.configPIDF(slot, p, i, d, 0.0)
    
    def from_slot0_noF(self, p: float, i: float, d: float):
        return self.configPIDF(0, p, i, d, 0.0)
    
    def addFollower(self, device_id: int, canbus: str, invertType: InvertedValue | bool):
        follower = TalonFX(device_id, canbus)
        if invertType in [InvertedValue.CLOCKWISE_POSITIVE, InvertedValue.COUNTER_CLOCKWISE_POSITIVE]:
            conf = TalonFXConfiguration()
            conf.motor_output.inverted = invertType
            follower.set_control(StrictFollower(self.motor.device_id))
        else:
            follower.set_control(Follower(self.motor.device_id, bool(invertType)))

        self.followers.append(follower)
    
    def without_canbus(self, device_id: int, invertType: InvertedValue | bool):
        self.addFollower(device_id, "", invertType)
    
    