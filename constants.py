from commands2.button import CommandPS4Controller


class FLConstants:
    DRIVE = 12
    TURN = 10
    CAN = 11

class FRConstants:
    DRIVE = 3
    TURN = 1
    CAN = 2

class BLConstants:
    DRIVE = 9
    TURN = 7
    CAN = 8

class BRConstants:
    DRIVE = 6
    TURN = 4
    CAN = 5

GYRO = 13

DRIVER_CONTROLLER = CommandPS4Controller(0)
RESET_POSE = DRIVER_CONTROLLER.triangle()

ELEVATOR_1 = 0
ELEVATOR_2 = 0