"""Command service for applying drive commands."""

import logging
from typing import Optional

from ..domain.models import RobotDriveCommand
from .factory import create_motor_controller
from .interfaces import MotorController

logger = logging.getLogger(__name__)


class CommandService:
    """Applies drive commands to the motor controller."""

    def __init__(self, controller: MotorController) -> None:
        self._controller = controller
        self._last_command: Optional[RobotDriveCommand] = None

    @property
    def last_command(self) -> Optional[RobotDriveCommand]:
        """Last received drive command."""
        return self._last_command

    def apply_drive_command(self, command: RobotDriveCommand) -> None:
        """Apply a drive command to the motor controller."""
        self._controller.set_speeds(command.left, command.right)
        self._last_command = command

    def stop(self) -> None:
        """Stop the robot."""
        logger.info("Stopping motors (disconnect or idle)")
        self._controller.stop()


_service: Optional[CommandService] = None


def get_command_service() -> CommandService:
    """Get or create the singleton command service."""
    global _service
    if _service is None:
        _service = CommandService(create_motor_controller())
    return _service
