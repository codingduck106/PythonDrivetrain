from dataclasses import dataclass
from wpiutil.wpistruct.dataclass import make_wpistruct
from wpimath.geometry import Pose2d

@make_wpistruct
@dataclass
class SwerveModuleStateStruct:
    speed: float
    angle: float

@make_wpistruct
@dataclass
class Pose2dStruct:
    x: float
    y: float
    rotation: float

@make_wpistruct
@dataclass
class Rotation3dStruct:
    x: float
    y: float
    z: float