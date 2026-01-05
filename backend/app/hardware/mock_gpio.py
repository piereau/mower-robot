"""Mock GPIO implementation for development without hardware."""

import logging
from typing import Dict

from .interfaces import GPIOInterface, PinMode, PinState


logger = logging.getLogger(__name__)


class MockGPIO(GPIOInterface):
    """Mock GPIO implementation that simulates pin states in memory.
    
    Used for development and testing when real hardware is not available.
    """
    
    def __init__(self) -> None:
        self._pins: Dict[int, Dict[str, any]] = {}
        logger.info("MockGPIO initialized")
    
    def setup(self, pin: int, mode: PinMode) -> None:
        """Configure a mock GPIO pin."""
        self._pins[pin] = {"mode": mode, "state": 0}
        logger.debug(f"Pin {pin} configured as {mode}")
    
    def read(self, pin: int) -> PinState:
        """Read mock pin state."""
        if pin not in self._pins:
            logger.warning(f"Reading unconfigured pin {pin}, returning 0")
            return 0
        
        state = self._pins[pin]["state"]
        logger.debug(f"Pin {pin} read: {state}")
        return state
    
    def write(self, pin: int, state: PinState) -> None:
        """Write to mock pin."""
        if pin not in self._pins:
            logger.warning(f"Writing to unconfigured pin {pin}")
            self._pins[pin] = {"mode": "OUT", "state": state}
        else:
            self._pins[pin]["state"] = state
        
        logger.debug(f"Pin {pin} written: {state}")
    
    def cleanup(self) -> None:
        """Clear all mock pin configurations."""
        self._pins.clear()
        logger.info("MockGPIO cleaned up")
    
    def set_mock_state(self, pin: int, state: PinState) -> None:
        """Test helper: set a pin state for simulation purposes.
        
        Args:
            pin: GPIO pin number
            state: State to simulate (0 or 1)
        """
        if pin in self._pins:
            self._pins[pin]["state"] = state
        else:
            self._pins[pin] = {"mode": "IN", "state": state}

