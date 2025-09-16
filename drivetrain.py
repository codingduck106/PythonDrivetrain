import math
from phoenix6.swerve import (
    SwerveDrivetrain, 
    SwerveDrivetrainConstants, 
    SwerveModuleConstants,
    ClosedLoopOutputType, 
    SteerFeedbackType
)
from phoenix6.hardware import TalonFX, CANcoder, Pigeon2
from phoenix6.configs import TalonFXConfiguration, CANcoderConfiguration, CurrentLimitsConfigs
from phoenix6.controls import VelocityVoltage, PositionVoltage, StaticBrake
from commands2 import Subsystem
import wpilib
import wpimath.kinematics
import wpimath.geometry
from ntcore import NetworkTableInstance
from constants import *

# Constants - adjust these to match your robot
kMaxSpeed = 6.0  # Phoenix 6 typically uses higher speeds (m/s)
kMaxAngularSpeed = 2 * math.pi  # rad/sec
kWheelRadius = 0.0390398  # meters
kDriveGearRatio = 6.75
kTurnGearRatio = 150.0 / 7
FRONT_DIST = 0.4064
LEFT_DIST = 0.3302

class Drivetrain(Subsystem):
    """
    Phoenix 6 swerve drivetrain with direct motor control instead of requests.
    """

    def __init__(self) -> None:
        """Initialize the Phoenix 6 swerve drivetrain."""
        super().__init__()
        
        # Create drivetrain constants
        self.drivetrain_constants = (
            SwerveDrivetrainConstants()
            .with_can_bus_name("swerve")
            .with_pigeon2_id(GYRO)
        )
        
        # Create module constants for all 4 modules
        module_constants = [
            self.create_module_constants(
                FLConstants.DRIVE, FLConstants.TURN, FLConstants.CAN, 
                FLConstants.OFFSET
            ),
            self.create_module_constants(
                FRConstants.DRIVE, FRConstants.TURN, FRConstants.CAN,
                FRConstants.OFFSET
            ),
            self.create_module_constants(
                BLConstants.DRIVE, BLConstants.TURN, BLConstants.CAN,
                BLConstants.OFFSET
            ),
            self.create_module_constants(
                BRConstants.DRIVE, BRConstants.TURN, BRConstants.CAN,
                BRConstants.OFFSET
            )
        ]
        
        # Create the Phoenix 6 swerve drivetrain
        self.drivetrain = SwerveDrivetrain(
            TalonFX, TalonFX, CANcoder,
            self.drivetrain_constants,
            module_constants
        )
        
        # Apply current limits to all drive motors
        current_limits = (
            CurrentLimitsConfigs()
            .with_supply_current_limit(60)
            .with_supply_current_limit_enable(True)
            .with_stator_current_limit_enable(False)
        )
        
        for i in range(4):
            self.drivetrain.get_module(i).drive_motor.configurator.apply(current_limits)
        
        # Create control objects for direct motor control
        self.drive_velocity_control = VelocityVoltage(0)
        self.steer_position_control = PositionVoltage(0)
        self.brake_control = StaticBrake()
        
        # Create WPILib kinematics for manual control
        self.front_left_location = wpimath.geometry.Translation2d(FRONT_DIST, LEFT_DIST)
        self.front_right_location = wpimath.geometry.Translation2d(FRONT_DIST, -LEFT_DIST)
        self.back_left_location = wpimath.geometry.Translation2d(-FRONT_DIST, LEFT_DIST)
        self.back_right_location = wpimath.geometry.Translation2d(-FRONT_DIST, -LEFT_DIST)
        
        self.kinematics = wpimath.kinematics.SwerveDrive4Kinematics(
            self.front_left_location,
            self.front_right_location,
            self.back_left_location,
            self.back_right_location
        )
        
        # NetworkTables for telemetry
        nt_instance = NetworkTableInstance.getDefault()
        self.nt_table = nt_instance.getTable("SmartDashboard")

    def create_module_constants(self, drive_id: int, turn_id: int, encoder_id: int, encoder_offset: float):
        """Create module constants for a single swerve module."""
        
        # Drive motor configuration
        drive_config = TalonFXConfiguration()
        drive_config.slot0.k_p = 3.0
        drive_config.slot0.k_i = 0.0
        drive_config.slot0.k_d = 0.0
        drive_config.slot0.k_s = 0.1
        drive_config.slot0.k_v = 0.12
        drive_config.slot0.k_a = 0.0
        
        # Turn motor configuration - LOWERED GAINS
        turn_config = TalonFXConfiguration()
        turn_config.slot0.k_p = 20.0  # Much lower than 100
        turn_config.slot0.k_i = 0.0
        turn_config.slot0.k_d = 0.0   # Start with 0 D term
        turn_config.slot0.k_s = 0.0
        turn_config.slot0.k_v = 1.5
        turn_config.slot0.k_a = 0.0
        
        # CANCoder configuration
        encoder_config = CANcoderConfiguration()
        encoder_config.magnet_sensor.absolute_sensor_discontinuity_point = 1
        encoder_config.magnet_sensor.magnet_offset = encoder_offset
        
        return (
            SwerveModuleConstants()
            .with_drive_motor_id(drive_id)
            .with_steer_motor_id(turn_id)
            .with_encoder_id(encoder_id)
            .with_drive_motor_gear_ratio(kDriveGearRatio)
            .with_steer_motor_gear_ratio(kTurnGearRatio) 
            .with_wheel_radius(kWheelRadius)
            .with_drive_motor_initial_configs(drive_config)
            .with_steer_motor_initial_configs(turn_config)
            .with_encoder_initial_configs(encoder_config)
            .with_speed_at12_volts(kMaxSpeed)
            .with_slip_current(800)
            .with_drive_motor_closed_loop_output(ClosedLoopOutputType.VOLTAGE)
            .with_steer_motor_closed_loop_output(ClosedLoopOutputType.VOLTAGE)
            .with_steer_motor_inverted(False)
            .with_drive_motor_inverted(False)
            .with_feedback_source(SteerFeedbackType.REMOTE_CANCODER)
            .with_coupling_gear_ratio(3.5)
            .with_steer_motor_gains(turn_config.slot0)
            .with_drive_motor_gains(drive_config.slot0)
        )

    # ====================== DRIVING METHODS (DIRECT MOTOR CONTROL) ======================
    
    def drive(self, x_speed: float, y_speed: float, rotation: float, 
              field_relative: bool = True, rate_limit: bool = True):
        """
        Drive using direct motor control instead of requests.
        """
        # Apply deadband
        DEADBAND = 0.1
        if abs(x_speed) < DEADBAND:
            x_speed = 0
        if abs(y_speed) < DEADBAND:
            y_speed = 0
        if abs(rotation) < DEADBAND:
            rotation = 0
        
        # If no movement, stop modules
        if x_speed == 0 and y_speed == 0 and rotation == 0:
            self.stop()
            return
        
        # Scale inputs to actual speeds
        vx = x_speed * kMaxSpeed
        vy = y_speed * kMaxSpeed
        omega = rotation * kMaxAngularSpeed
        
        # Create chassis speeds
        if field_relative:
            chassis_speeds = wpimath.kinematics.ChassisSpeeds.fromFieldRelativeSpeeds(
                vx, vy, omega, self.get_rotation()
            )
        else:
            chassis_speeds = wpimath.kinematics.ChassisSpeeds(vx, vy, omega)
        
        # Convert to module states
        module_states = self.kinematics.toSwerveModuleStates(chassis_speeds)
        wpimath.kinematics.SwerveDrive4Kinematics.desaturateWheelSpeeds(module_states, kMaxSpeed)
        
        # Set each module directly
        self.set_module_states(module_states)

    def set_module_states(self, desired_states):
        """Set module states using direct motor control."""
        for i, desired_state in enumerate(desired_states):
            module = self.drivetrain.get_module(i)
            
            # Optimize the state to minimize rotation
            current_angle = module.get_current_state().angle
            optimized_state = self.optimize(desired_state, current_angle)
            
            # Get motors
            drive_motor = module.drive_motor
            steer_motor = module.steer_motor
            
            # Set drive motor velocity (m/s to rotations/s conversion)
            drive_rps = optimized_state.speed / (2 * math.pi * kWheelRadius) * kDriveGearRatio
            drive_motor.set_control(self.drive_velocity_control.with_velocity(drive_rps))
            
            # Set steer motor position (radians to rotations conversion)
            steer_rotations = optimized_state.angle.radians() / (2 * math.pi) * kTurnGearRatio
            steer_motor.set_control(self.steer_position_control.with_position(steer_rotations))

    def optimize(self, desired_state, current_angle):
        """Optimize module state to minimize rotation."""
        # Get the difference between desired and current angle
        delta = desired_state.angle.radians() - current_angle.radians()
        
        # Normalize to [-pi, pi]
        while delta > math.pi:
            delta -= 2 * math.pi
        while delta < -math.pi:
            delta += 2 * math.pi
        
        # If the difference is greater than 90 degrees, reverse speed and add 180°
        if abs(delta) > math.pi / 2:
            return wpimath.kinematics.SwerveModuleState(
                -desired_state.speed,
                wpimath.geometry.Rotation2d(desired_state.angle.radians() + math.pi)
            )
        else:
            return desired_state

    def drive_chassis_speeds(self, speeds):
        """Drive using chassis speeds for autonomous."""
        # Convert to module states
        module_states = self.kinematics.toSwerveModuleStates(speeds)
        wpimath.kinematics.SwerveDrive4Kinematics.desaturateWheelSpeeds(module_states, kMaxSpeed)

        # Set each module directly
        self.set_module_states(module_states)

    def stop(self):
        """Stop all modules using brake control."""
        for i in range(4):
            module = self.drivetrain.get_module(i)
            module.drive_motor.set_control(self.brake_control)
            # Don't brake the steer motor - let it hold position
            # module.steer_motor.set_control(self.brake_control)

    def point_wheels(self, angle_radians: float):
        """Point all wheels in a specific direction."""
        target_angle = wpimath.geometry.Rotation2d(angle_radians)
        zero_speed_state = wpimath.kinematics.SwerveModuleState(0.0, target_angle)
        
        for i in range(4):
            module = self.drivetrain.get_module(i)
            drive_motor = module.drive_motor
            steer_motor = module.steer_motor
            
            # Stop drive motor
            drive_motor.set_control(self.brake_control)
            
            # Point steer motor to desired angle
            steer_rotations = angle_radians / (2 * math.pi) * kTurnGearRatio
            steer_motor.set_control(self.steer_position_control.with_position(steer_rotations))

    def x_stance(self):
        """Put wheels in X formation."""
        angles = [math.pi/4, -math.pi/4, -math.pi/4, math.pi/4]  # FL, FR, BL, BR
        
        for i, angle in enumerate(angles):
            module = self.drivetrain.get_module(i)
            drive_motor = module.drive_motor
            steer_motor = module.steer_motor
            
            # Stop drive motor
            drive_motor.set_control(self.brake_control)
            
            # Point to X formation angle
            steer_rotations = angle / (2 * math.pi) * kTurnGearRatio
            steer_motor.set_control(self.steer_position_control.with_position(steer_rotations))

    # ====================== GETTERS ======================

    def get_pose(self):
        """Get current robot pose."""
        return self.drivetrain.get_state().pose

    def get_rotation(self):
        """Get current robot rotation."""
        return self.drivetrain.pigeon2.getRotation2d()

    def get_chassis_speeds(self):
        """Get current chassis speeds."""
        module_states = [self.drivetrain.get_module(i).get_current_state() for i in range(4)]
        return self.kinematics.toChassisSpeeds(tuple(module_states)) # type: ignore

    def get_module_states(self):
        """Get current module states."""
        return [self.drivetrain.get_module(i).get_current_state() for i in range(4)]

    def get_module_positions(self):
        """Get current module positions.""" 
        return [self.drivetrain.get_module(i).get_position(True) for i in range(4)]

    # ====================== ODOMETRY ======================

    def reset_pose(self, pose=None):
        """Reset odometry to specified pose."""
        if pose is None:
            pose = wpimath.geometry.Pose2d()
        self.drivetrain.reset_pose(pose)

    def add_vision_measurement(self, pose, timestamp, std_devs: tuple[float, float, float] | None = None):
        """Add vision measurement to pose estimator."""
        if std_devs is None:
            std_devs = (0.1, 0.1, 0.1)
        self.drivetrain.add_vision_measurement(pose, timestamp, tuple(std_devs)) # type: ignore

    # ====================== UTILITIES ======================

    def should_flip_path(self):
        """Determine if autonomous paths should be flipped."""
        alliance = wpilib.DriverStation.getAlliance()
        return alliance is not None and alliance == wpilib.DriverStation.Alliance.kRed

    # ====================== PERIODIC METHODS ======================

    def periodic(self):
        """Called periodically by the command scheduler."""
        # Phoenix 6 handles odometry updates automatically
        self.publish_telemetry()

    def publish_telemetry(self):
        """Publish telemetry to NetworkTables."""
        try:
            state = self.drivetrain.get_state()
            
            # Publish pose
            pose = state.pose
            self.nt_table.putNumberArray("Pose", [pose.X(), pose.Y(), pose.rotation().degrees()])
            
            # Publish module states
            module_data = []
            for module_state in state.module_states:
                module_data.extend([module_state.speed, module_state.angle.degrees()])
            self.nt_table.putNumberArray("ModuleStates", module_data)
            
            # Publish other useful data
            speeds = self.get_chassis_speeds()
            self.nt_table.putNumber("RobotSpeed", speeds.vx)
            self.nt_table.putNumber("RobotRotation", self.get_rotation().degrees())
        except Exception as e:
            print(f"Telemetry error: {e}")

    # ====================== CALIBRATION ======================

    def calibrate_offsets(self):
        """Helper method to calibrate encoder offsets."""
        print("=== ENCODER OFFSET CALIBRATION ===")
        print("Make sure all wheels are pointing straight forward!")
        input("Press Enter when ready...")
        
        offsets = []
        for i in range(4):
            module = self.drivetrain.get_module(i)
            # Get the CANCoder directly for calibration
            encoder = module.encoder
            current_position = encoder.get_absolute_position().value
            
            # Calculate offset to make this position = 0
            offset = -current_position
            if offset < -0.5:
                offset += 1.0
            elif offset > 0.5:
                offset -= 1.0
                
            offsets.append(offset)
            print(f"Module {i} offset: {offsets[i]:.6f}")
        
        print("\nAdd these to your constants file:")
        module_names = ["FL", "FR", "BL", "BR"]
        for name, offset in zip(module_names, offsets):
            print(f"{name}Constants.OFFSET = {offset:.6f}")
        
        return offsets