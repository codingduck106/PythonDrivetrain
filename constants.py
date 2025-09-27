from commands2.button import CommandPS4Controller

# This is just the constants the robot uses


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
SIM_STEER_CONTROLER = CommandPS4Controller(1)
RESET_POSE = DRIVER_CONTROLLER.triangle()