from commands2.button import CommandPS4Controller


class FLConstants:
    DRIVE = 12
    TURN = 10
    CAN = 11
    OFFSET = 0.0

class FRConstants:
    DRIVE = 3
    TURN = 1
    CAN = 2
    OFFSET = 0.0 # negated phoenix tuner x value

class BLConstants:
    DRIVE = 9
    TURN = 7
    CAN = 8
    OFFSET = 0.0

class BRConstants:
    DRIVE = 6
    TURN = 4
    CAN = 5
    OFFSET = 0.0

GYRO = 13

DRIVER_CONTROLLER = CommandPS4Controller(0)
RESET_POSE = DRIVER_CONTROLLER.triangle()