"""Factory for motor controller selection."""

from ..settings import settings
from .interfaces import MotorController
from .mock_controller import MockMotorController
from .serial_controller import SerialMotorController


def create_motor_controller() -> MotorController:
    """Create motor controller based on settings."""
    if settings.use_mock_motor_controller:
        return MockMotorController()

    return SerialMotorController(
        port=settings.serial_port,
        baud=settings.serial_baud,
    )
