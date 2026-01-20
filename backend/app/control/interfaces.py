"""Motor controller interfaces."""

from abc import ABC, abstractmethod


class MotorController(ABC):
    """Abstract interface for motor control."""

    @abstractmethod
    def set_speeds(self, left: float, right: float) -> None:
        """Set motor speeds.

        Args:
            left: Left motor speed (-1.0 to 1.0)
            right: Right motor speed (-1.0 to 1.0)
        """
        pass

    @abstractmethod
    def stop(self) -> None:
        """Stop both motors."""
        pass
