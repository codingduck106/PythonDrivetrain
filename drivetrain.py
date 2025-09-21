# import math
# from phoenix6.swerve import (
#     SwerveDrivetrain, 
#     SwerveDrivetrainConstants, 
#     SwerveModuleConstants,
#     ClosedLoopOutputType, 
#     SteerFeedbackType
# )
# from phoenix6.hardware import TalonFX, CANcoder, Pigeon2
# from phoenix6.configs import TalonFXConfiguration, CANcoderConfiguration, CurrentLimitsConfigs
# from commands2 import Subsystem
# from phoenix6.controls import StaticBrake
# from phoenix6.swerve.requests import *
# import wpilib
# from ntcore import NetworkTableInstance
# from constants import *

# # Constants - adjust these to match your robot
# kMaxSpeed = 6.0  # m/s
# kMaxAngularSpeed = 2 * math.pi  # rad/sec
# kWheelRadius = 0.0390398  # meters
# kDriveGearRatio = 6.75
# kTurnGearRatio = 150.0 / 7
# FRONT_DIST = 0.4064
# LEFT_DIST = 0.3302

# class Drivetrain(Subsystem):
#     """
#     Phoenix 6 swerve drivetrain using the requests API.
#     """

#     def __init__(self) -> None:
#         """Initialize the Phoenix 6 swerve drivetrain."""
        
#         # Create drivetrain constants
#         self.drivetrain_constants = (
#             SwerveDrivetrainConstants()
#             .with_can_bus_name("swerve")
#             .with_pigeon2_id(GYRO)
#         )
        
#         # Create module constants for all 4 modules
#         module_constants = [
#             self.create_module_constants(
#                 FLConstants.DRIVE, FLConstants.TURN, FLConstants.CAN, 
#                 FLConstants.OFFSET
#             ),
#             self.create_module_constants(
#                 FRConstants.DRIVE, FRConstants.TURN, FRConstants.CAN,
#                 FRConstants.OFFSET
#             ),
#             self.create_module_constants(
#                 BLConstants.DRIVE, BLConstants.TURN, BLConstants.CAN,
#                 BLConstants.OFFSET
#             ),
#             self.create_module_constants(
#                 BRConstants.DRIVE, BRConstants.TURN, BRConstants.CAN,
#                 BRConstants.OFFSET
#             )
#         ]
        
#         # Create the Phoenix 6 swerve drivetrain
#         self.drivetrain = SwerveDrivetrain(
#             TalonFX, TalonFX, CANcoder,
#             self.drivetrain_constants,
#             module_constants
#         )
        
#         # Apply current limits to all drive motors
#         current_limits = (
#             CurrentLimitsConfigs()
#             .with_supply_current_limit(60)
#             .with_supply_current_limit_enable(True)
#             .with_stator_current_limit_enable(False)
#         )
        
#         for i in range(4):
#             self.drivetrain.get_module(i).drive_motor.configurator.apply(current_limits)
        
#         # ===== SWERVE REQUESTS - THESE ARE THE KEY PART! =====
        
#         # Create request objects once and reuse them
#         self.field_centric_applier = FieldCentric().with_drive_request_type(SwerveModule.DriveRequestType.VELOCITY)
#         self.robot_centric_applier = RobotCentric()
#         self.field_centric_facing_applier = FieldCentricFacingAngle()
        
#         # For autonomous - uses ChassisSpeeds directly
#         self.auto_request = ApplyRobotSpeeds()
        
#         # For SysId characterization (if needed)
#         self.sysid_translation_request = SysIdSwerveTranslation()
#         self.sysid_rotation_request = SysIdSwerveRotation()
#         self.sysid_steer_request = SysIdSwerveSteerGains()
        
#         # NetworkTables for telemetry
#         nt_instance = NetworkTableInstance.getDefault()
#         self.nt_table = nt_instance.getTable("SmartDashboard")
#         self.correct_rotation = False

#     def on_red(self):
#         return True if wpilib.DriverStation.getAlliance() == wpilib.DriverStation.Alliance.kRed else False
    

#     def get_rotation_3d(self):
#         return self.drivetrain.get_rotation3d()
    
#     def modifyAxis(self, value: float):
#         if abs(value) <= 0.1:
#             value = 0
#         else:
#             if value > 0.0:
#                 value =  (value - 0.1) / (1.0 - 0.1)
#             else:
#                 value =  (value + 0.1) / (1.0 - 0.1)

#         value = math.copysign(value * value, value)
#         return value
    
#     def get_position(self, module_id: int):
#         return (self.drivetrain
#                 .get_module(module_id)
#                 .encoder
#                 .get_absolute_position()
#                 .value)

#     def set_rotation_velocity(self, rotation_rate: rotations_per_second):
#         self.drivetrain.set_control(self.field_centric_applier.with_rotational_rate(rotation_rate))

#     def get_robot_relative_speeds(self):
#         return self.drivetrain.kinematics.toChassisSpeeds(tuple(self.drivetrain.get_state().module_states)) # type: ignore
    
#     def enable_slowmode(self, enable: bool):
#         self.multiplier = 0.3 if enable else 1

#     def close(self):
#         self.drivetrain.close() 

#     def create_module_constants(self, drive_id: int, turn_id: int, encoder_id: int, encoder_offset: float):
#         """Create module constants for a single swerve module."""
        
#         # Drive motor configuration - matching Java gains
#         drive_config = TalonFXConfiguration()
#         drive_config.slot0.k_p = 2.5
#         drive_config.slot0.k_i = 0.0
#         drive_config.slot0.k_d = 0.0
#         drive_config.slot0.k_s = 6.4111
#         drive_config.slot0.k_v = 0.087032
#         drive_config.slot0.k_a = 0.0
        
#         # Turn motor configuration - matching Java gains
#         turn_config = TalonFXConfiguration()
#         turn_config.slot0.k_p = 50.0  # Match Java
#         turn_config.slot0.k_i = 0.0
#         turn_config.slot0.k_d = 3.0889  # Match Java
#         turn_config.slot0.k_s = 0.21041
#         turn_config.slot0.k_v = 2.68
#         turn_config.slot0.k_a = 0.084645
        
#         # CANCoder configuration
#         encoder_config = CANcoderConfiguration()
#         encoder_config.magnet_sensor.absolute_sensor_discontinuity_point = 1
#         encoder_config.magnet_sensor.magnet_offset = encoder_offset
        
#         return (
#             SwerveModuleConstants()
#             .with_drive_motor_id(drive_id)
#             .with_steer_motor_id(turn_id)
#             .with_encoder_id(encoder_id)
#             .with_drive_motor_gear_ratio(kDriveGearRatio)
#             .with_steer_motor_gear_ratio(kTurnGearRatio) 
#             .with_wheel_radius(kWheelRadius)
#             .with_drive_motor_initial_configs(drive_config)
#             .with_steer_motor_initial_configs(turn_config)
#             .with_encoder_initial_configs(encoder_config)
#             .with_speed_at12_volts(kMaxSpeed)
#             .with_slip_current(800)
#             .with_drive_motor_closed_loop_output(ClosedLoopOutputType.VOLTAGE)
#             .with_steer_motor_closed_loop_output(ClosedLoopOutputType.VOLTAGE)
#             .with_steer_motor_inverted(False)
#             .with_drive_motor_inverted(False)
#             .with_feedback_source(SteerFeedbackType.REMOTE_CANCODER)
#             .with_coupling_gear_ratio(3.5)
#             .with_steer_motor_gains(turn_config.slot0)
#             .with_drive_motor_gains(drive_config.slot0)
#         )

#     # ====================== DRIVING METHODS (USING REQUESTS API) ======================
    
#     def drive(self, x_speed: float, y_speed: float, rotation: float, field_relative: bool = True):
#         """
#         Drive using Phoenix 6 requests API - matches your working Java version.
        
#         Args:
#             x_speed: Forward/backward speed (-1 to 1)
#             y_speed: Left/right speed (-1 to 1) 
#             rotation: Rotation speed (-1 to 1)
#             field_relative: Whether to drive relative to field or robot
#         """
#         # Apply deadband and squaring like Java version
#         x_speed = self.modify_axis(x_speed)
#         y_speed = self.modify_axis(y_speed)
#         rotation = self.modify_axis(rotation)
        
#         # Scale inputs to actual speeds
#         vx = x_speed * kMaxSpeed
#         vy = y_speed * kMaxSpeed
#         omega = rotation * kMaxAngularSpeed
        
#         # Use field-centric driving (matching Java)
#         request = (self.field_centric_applier.with_velocity_x(vx)
#                    .with_velocity_y(vy)
#                    .with_rotational_rate(omega))
        
#         # Send the request to the drivetrain - Phoenix 6 handles everything else!
#         self.drivetrain.set_control(request)
    
#     def modify_axis(self, value: float) -> float:
#         """Apply deadband and squaring like the Java version."""
#         # Apply deadband
#         DEADBAND = 0.1
#         if abs(value) < DEADBAND:
#             value = 0.0
        
#         # Square the input while preserving sign (like Java)
#         value = math.copysign(value * value, value)
        
#         return value

#     def drive_chassis_speeds(self, chassis_speeds: ChassisSpeeds):
#         """
#         Drive using chassis speeds for autonomous.
        
#         Args:
#             chassis_speeds: wpimath.kinematics.ChassisSpeeds object
#         """
#         request = self.auto_request.with_speeds(chassis_speeds)
#         self.drivetrain.set_control(request)

#     def stop(self):
#         """Stop the drivetrain by going to idle."""
#         self.drivetrain.set_control(Idle())

#     def point_wheels(self, direction: Rotation2d):
#         """
#         Point all wheels in a specific direction.
        
#         Args:
#             direction: wpimath.geometry.Rotation2d object
#         """
#         request = self.field_centric_facing_applier.with_target_direction(direction)
#         self.drivetrain.set_control(request)

#     # ====================== ADVANCED REQUESTS ======================
    
#     def drive_with_path_following(self, target_speeds: ChassisSpeeds, target_pose: Pose2d):
#         """
#         Example of more advanced request usage with feedforward.
#         This would be used in autonomous path following.
#         """
#         request = (self.auto_request
#                   .with_speeds(target_speeds)
#                   .with_wheel_force_feedforwards_x([0, 0, 0, 0])  # Optional feedforward
#                   .with_wheel_force_feedforwards_y([0, 0, 0, 0]))
        
#         self.drivetrain.set_control(request)

#     def characterize_translation(self, volts: float):
#         """Run translation characterization for SysId."""
#         request = self.sysid_translation_request.with_volts(volts)
#         self.drivetrain.set_control(request)

#     def characterize_rotation(self, volts: float):
#         """Run rotation characterization for SysId.""" 
#         request = self.sysid_rotation_request.with_rotational_rate(volts)
#         self.drivetrain.set_control(request)

#     def characterize_steer(self, volts: float):
#         """Run steer characterization for SysId."""
#         request = self.sysid_steer_request.with_volts(volts)
#         self.drivetrain.set_control(request)

#     # ====================== GETTERS (SAME AS BEFORE) ======================

#     def get_pose(self):
#         """Get current robot pose."""
#         return self.drivetrain.get_state().pose

#     def get_rotation(self):
#         """Get current robot rotation."""
#         return self.drivetrain.pigeon2.getRotation2d()

#     def get_chassis_speeds(self):
#         """Get current chassis speeds."""
#         return self.drivetrain.kinematics.toChassisSpeeds(tuple(self.drivetrain.get_state().module_states)) # type: ignore

#     def get_module_states(self):
#         """Get current module states."""
#         return self.drivetrain.get_state().module_states

#     def get_module_positions(self):
#         """Get current module positions.""" 
#         return self.drivetrain.get_state().module_positions

#     # ====================== ODOMETRY ======================

#     def reset_pose(self):
#         self.correct_rotation = False
#         self.drivetrain.seed_field_centric()

#     def set_pose(self, pose: Pose2d):
#         """Reset odometry to specified pose."""
#         self.correct_rotation = True
#         if pose is not None:
#             self.drivetrain.reset_pose(pose)
#         else:
#             raise RuntimeError("set_pose() was passed a None argument! Possibly unintended behavior may occur.", False)

#     def add_vision_measurement(self, pose: Pose2d, timestamp: second, std_devs: tuple[float, float, float] | None = None):
#         """Add vision measurement to pose estimator."""
#         if std_devs is None:
#             std_devs = (0.1, 0.1, 0.1)
#         self.drivetrain.add_vision_measurement(pose, timestamp, tuple(std_devs)) # type: ignore

#     # ====================== UTILITIES ======================

#     def should_flip_path(self):
#         """Determine if autonomous paths should be flipped."""
#         alliance = wpilib.DriverStation.getAlliance()
#         return alliance is not None and alliance == wpilib.DriverStation.Alliance.kRed

#     # ====================== PERIODIC METHODS ======================

#     def periodic(self):
#         """Called periodically by the command scheduler."""
#         # Phoenix 6 handles odometry updates automatically
#         self.publish_telemetry()

#     def publish_telemetry(self):
#         """Publish telemetry to NetworkTables."""
#         try:
#             state = self.drivetrain.get_state()
            
#             # Publish pose
#             pose = state.pose
#             self.nt_table.putNumberArray("Pose", [pose.X(), pose.Y(), pose.rotation().degrees()])
            
#             # Publish module states
#             module_data = []
#             for module_state in state.module_states:
#                 module_data.extend([module_state.speed, module_state.angle.degrees()])
#             self.nt_table.putNumberArray("ModuleStates", module_data)
            
#             # Publish other useful data
#             speeds = self.get_chassis_speeds()
#             self.nt_table.putNumber("RobotSpeed", speeds.vx)
#             self.nt_table.putNumber("RobotRotation", self.get_rotation().degrees())
#         except Exception as e:
#             print(f"Telemetry error: {e}")

#     # ====================== CALIBRATION ======================

#     def calibrate_offsets(self):
#         """Helper method to calibrate encoder offsets."""
#         print("=== ENCODER OFFSET CALIBRATION ===")
#         print("Make sure all wheels are pointing straight forward!")
#         input("Press Enter when ready...")
        
#         offsets = []
#         for i in range(4):
#             module = self.drivetrain.get_module(i)
#             encoder = module.encoder
#             current_position = encoder.get_absolute_position().value
            
#             # Calculate offset to make this position = 0
#             offset = -current_position
#             if offset < -0.5:
#                 offset += 1.0
#             elif offset > 0.5:
#                 offset -= 1.0
                
#             offsets.append(offset)
#             print(f"Module {i} offset: {offsets[i]:.6f}")
        
#         print("\nAdd these to your constants file:")
#         module_names = ["FL", "FR", "BL", "BR"]
#         for name, offset in zip(module_names, offsets):
#             print(f"{name}Constants.OFFSET = {offset:.6f}")
        
#         return offsets