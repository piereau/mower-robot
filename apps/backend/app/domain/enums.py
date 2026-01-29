"""Domain enums for robot state."""

from enum import Enum


class RobotState(str, Enum):
    """Possible states of the robot."""
    
    IDLE = "idle"
    MOWING = "mowing"
    ERROR = "error"

