# from pathplannerlib.config import RobotConfig, PIDConstants
# from pathplannerlib.auto import AutoBuilder
# from pathplannerlib.controller import PPHolonomicDriveController


from drivetrain import *
from ntcore import NetworkTableInstance
from wpilib import SmartDashboard
from commands2.instantcommand import InstantCommand
from constants import *

class RobotContainer:
    """Fixed robot container to prevent command conflicts"""
    
    def __init__(self, alliance: wpilib.DriverStation.Alliance | None):
        """Initialize with conflict prevention"""
        self.drive = Drivetrain()  # Use the fixed drivetrain
        self.configureBindings()
        
        # PathPlanner setup - but with better isolation
        # try:
        #     config = RobotConfig.fromGUISettings()
            
        #     AutoBuilder.configure(
        #         self.drive.get_pose,
        #         self.drive.reset_pose,
        #         self.drive.get_chassis_speeds,
        #         lambda speeds, ff: self.drive.drive_chassis_speeds(speeds),  # This calls the fixed method
        #         PPHolonomicDriveController(
        #             PIDConstants(5.0, 0.0, 0.0),
        #             PIDConstants(5.0, 0.0, 0.0),
        #         ),
        #         config,
        #         self.drive.should_flip_path,
        #         self.drive,
        #     )
            
        #     self.autoChooser = AutoBuilder.buildAutoChooser()
        # except Exception as e:
        #     print(f"PathPlanner setup failed: {e}")
        #     self.autoChooser = None
        
        # Reduced NetworkTables usage
        self.nt_table = NetworkTableInstance.getDefault().getTable("SmartDashboard")
        # if self.autoChooser:
        #     SmartDashboard.putData("Auto Chooser", self.autoChooser)
    
    def configureBindings(self):
        """Simplified bindings to prevent conflicts"""
        from constants import RESET_POSE
        from commands2.instantcommand import InstantCommand
        
        # Only bind essential commands
        RESET_POSE.onTrue(InstantCommand(lambda: self.drive.reset_pose()))
    
    # def getAutonomousCommand(self):
    #     """Get auto command with error handling"""
    #     if self.autoChooser:
    #         return self.autoChooser.getSelected()
    #     return None

# ====================== DEBUGGING METHODS ======================

def debug_control_conflicts():
    """Debug method to identify control conflicts"""
    
    print("""
    === CONTROL CONFLICT DEBUG CHECKLIST ===
    
    ✅ Common Causes of 70-degree oscillation:
    
    1. RAPID FIRE COMMANDS:
       - robotPeriodic() running telemetry at 50Hz
       - teleopPeriodic() sending drive commands at 50Hz  
       - Both accessing module states simultaneously
    
    2. PATHPLANNER INTERFERENCE:
       - AutoBuilder still active in teleop
       - Autonomous command not properly cancelled
       - PathPlanner sending conflicting chassis speeds
    
    3. COMMAND SCHEDULER CONFLICTS:
       - Multiple commands trying to control drivetrain
       - InstantCommand reset_pose running during drive
    
    4. NETWORKTABLES SPAM:
       - Publishing module states every 20ms
       - NetworkTables causing delays in command processing
    
    ✅ Solutions Applied:
    
    1. Rate limited drive commands (50Hz max)
    2. Reduced telemetry frequency (10Hz instead of 50Hz)
    3. Added proper teleop initialization  
    4. Prevented duplicate requests
    5. Added larger deadband (0.15 instead of 0.1)
    6. Cancelled auto commands on teleop init
    
    ✅ Test This:
    
    1. Deploy the fixed code above
    2. Enable teleop with NO joystick input
    3. Modules should NOT oscillate
    4. If still oscillating, the issue is in Phoenix 6 config
    """)
