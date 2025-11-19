from wpilib import TimedRobot
from robotcontainer import RobotContainer
from ntcore import NetworkTableInstance
from wpilib import DataLogManager
from commands2 import CommandScheduler
from phoenix6.signal_logger import SignalLogger

class Robot(TimedRobot):
    robotContainer = RobotContainer(NetworkTableInstance.getDefault())
    commandScheduler = CommandScheduler.getInstance()

    def robotInit(self) -> None:
        DataLogManager.start()
        logDir = DataLogManager.getLogDir()

        SignalLogger.set_path(logDir)

        SignalLogger.enable_auto_logging(True)

        ntInstance = NetworkTableInstance.getDefault()
        ntInstance.getTable("Metadata").getStringTopic("LogDir").publish().set(logDir)

    def robotPeriodic(self) -> None:
        self.commandScheduler.run()

    def testInit(self) -> None:
        self.commandScheduler.cancelAll()

