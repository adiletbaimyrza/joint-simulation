from enum import Enum


class JointType(Enum):
    NONE = "none"
    DISTANCE = "distance"
    HINGE = "hinge"
    SERVO = "servo"
    MOTOR = "motor"
    BALL = "ball"
    PRISMATIC = "prismatic"
    CYLINDER = "cylinder"
    FIXED = "fixed"
