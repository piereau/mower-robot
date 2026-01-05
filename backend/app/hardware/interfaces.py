"""Hardware interface contracts.

These abstract base classes define the contract for GPIO operations.
Implementations can be swapped between mock (development) and real (production).
"""

from abc import ABC, abstractmethod
from typing import Literal


PinMode = Literal["IN", "OUT"]
PinState = Literal[0, 1]


class GPIOInterface(ABC):
    """Abstract interface for GPIO operations."""
    
    @abstractmethod
    def setup(self, pin: int, mode: PinMode) -> None:
        """Configure a GPIO pin as input or output.
        
        Args:
            pin: GPIO pin number
            mode: Pin mode ("IN" for input, "OUT" for output)
        """
        pass
    
    @abstractmethod
    def read(self, pin: int) -> PinState:
        """Read the current state of a GPIO pin.
        
        Args:
            pin: GPIO pin number
            
        Returns:
            Current pin state (0 or 1)
        """
        pass
    
    @abstractmethod
    def write(self, pin: int, state: PinState) -> None:
        """Write a state to a GPIO pin.
        
        Args:
            pin: GPIO pin number
            state: State to write (0 or 1)
        """
        pass
    
    @abstractmethod
    def cleanup(self) -> None:
        """Release all GPIO resources."""
        pass

