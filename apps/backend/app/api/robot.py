"""Robot status REST endpoint."""

from fastapi import APIRouter

from ..domain.models import RobotTelemetry
from ..simulation.telemetry import get_simulator


router = APIRouter()


@router.get("/robot/status", response_model=RobotTelemetry)
async def get_robot_status() -> RobotTelemetry:
    """Get the current robot status.
    
    Returns the last known telemetry data without advancing the simulation.
    Useful for initial page load before WebSocket connects.
    
    Returns:
        Current robot telemetry snapshot
    """
    simulator = get_simulator()
    return simulator.get_telemetry()

