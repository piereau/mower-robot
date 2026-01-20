"""Mock motor controller for development."""

import logging

from .interfaces import MotorController

logger = logging.getLogger(__name__)


class MockMotorController(MotorController):
    """Mock controller that logs commands."""

    def __init__(self) -> None:
        self._left = 0.0
        self._right = 0.0
        logger.info("MockMotorController initialized")

    def set_speeds(self, left: float, right: float) -> None:
        self._left = left
        self._right = right
        logger.info("Mock motors set: left=%.2f right=%.2f", left, right)

    def stop(self) -> None:
        self.set_speeds(0.0, 0.0)
