"""Telemetry simulator for POC development.

Generates realistic-looking robot telemetry data without real hardware.
"""

import random
from datetime import datetime
from typing import Optional

from ..domain.enums import RobotState
from ..domain.models import RobotTelemetry


# Probability weights for state transitions
STATE_TRANSITION_WEIGHTS = {
    RobotState.IDLE: {
        RobotState.IDLE: 1.00,
        RobotState.MOWING: 0.00,
        RobotState.ERROR: 0.00,
    },
    RobotState.MOWING: {
        RobotState.IDLE: 1.00,
        RobotState.MOWING: 0.00,
        RobotState.ERROR: 0.00,
    },
    RobotState.ERROR: {
        RobotState.IDLE: 1.00,
        RobotState.MOWING: 0.00,
        RobotState.ERROR: 0.00,
    },
}

# Battery drain/charge rates per tick
BATTERY_CHANGE_RATES = {
    RobotState.IDLE: 0.1,      # Slow charge when idle
    RobotState.MOWING: -0.5,   # Drain when mowing
    RobotState.ERROR: -0.2,    # Slight drain on error
}


class TelemetrySimulator:
    """Generates simulated robot telemetry data.
    
    The simulator maintains internal state and produces realistic
    state transitions and battery level changes over time.
    """
    
    def __init__(
        self,
        initial_state: RobotState = RobotState.IDLE,
        initial_battery: int = 80,
    ) -> None:
        """Initialize the simulator.
        
        Args:
            initial_state: Starting robot state
            initial_battery: Starting battery percentage (0-100)
        """
        self._state = initial_state
        self._battery = float(initial_battery)
        self._tick_count = 0
    
    @property
    def current_state(self) -> RobotState:
        """Current robot state."""
        return self._state
    
    @property
    def current_battery(self) -> int:
        """Current battery level (rounded to int)."""
        return int(round(self._battery))
    
    def tick(self) -> RobotTelemetry:
        """Advance simulation by one tick and return new telemetry.
        
        Each tick:
        1. Potentially transitions state based on probabilities
        2. Updates battery level based on current state
        3. Returns the new telemetry snapshot
        
        Returns:
            Current telemetry after this tick
        """
        self._tick_count += 1
        
        # Update state
        self._state = self._get_next_state()
        
        # Update battery
        self._update_battery()
        
        # Force idle if battery is critically low
        if self._battery < 10 and self._state == RobotState.MOWING:
            self._state = RobotState.IDLE
        
        return self.get_telemetry()
    
    def get_telemetry(self) -> RobotTelemetry:
        """Get current telemetry without advancing simulation.
        
        Returns:
            Current telemetry snapshot
        """
        return RobotTelemetry(
            state=self._state,
            battery=self.current_battery,
            timestamp=datetime.utcnow(),
        )
    
    def _get_next_state(self) -> RobotState:
        """Determine next state based on transition probabilities."""
        weights = STATE_TRANSITION_WEIGHTS[self._state]
        states = list(weights.keys())
        probabilities = list(weights.values())
        
        return random.choices(states, weights=probabilities, k=1)[0]
    
    def _update_battery(self) -> None:
        """Update battery level based on current state."""
        rate = BATTERY_CHANGE_RATES[self._state]
        
        self._battery += rate
        
        # Clamp to valid range
        self._battery = max(0, min(100, self._battery))
    
    def reset(
        self,
        state: Optional[RobotState] = None,
        battery: Optional[int] = None,
    ) -> None:
        """Reset simulator to specified state.
        
        Args:
            state: New state (defaults to IDLE)
            battery: New battery level (defaults to 80)
        """
        self._state = state or RobotState.IDLE
        self._battery = float(battery if battery is not None else 80)
        self._tick_count = 0


# Global simulator instance (singleton for the app)
_simulator: Optional[TelemetrySimulator] = None


def get_simulator() -> TelemetrySimulator:
    """Get or create the global simulator instance."""
    global _simulator
    if _simulator is None:
        _simulator = TelemetrySimulator()
    return _simulator

