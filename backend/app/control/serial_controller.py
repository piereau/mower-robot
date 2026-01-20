"""Serial motor controller for Arduino Nano over USB."""

import logging

from .interfaces import MotorController

logger = logging.getLogger(__name__)


class SerialMotorController(MotorController):
    """Motor controller that sends commands over a serial port."""

    def __init__(self, port: str, baud: int) -> None:
        try:
            import serial  # type: ignore
        except ImportError as exc:
            raise RuntimeError(
                "pyserial is required for SerialMotorController. "
                "Install with: pip install pyserial"
            ) from exc

        self._serial = serial.Serial(port=port, baudrate=baud, timeout=0.1)
        logger.info("SerialMotorController connected on %s @ %d", port, baud)

    def set_speeds(self, left: float, right: float) -> None:
        message = f"L:{left:.3f},R:{right:.3f}\n"
        self._serial.write(message.encode("ascii"))
        logger.debug("Serial command sent: %s", message.strip())

    def stop(self) -> None:
        self.set_speeds(0.0, 0.0)
