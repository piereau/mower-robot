"""Domain models for robot telemetry."""

from datetime import datetime

from pydantic import BaseModel, Field

from .enums import RobotState


class RobotTelemetry(BaseModel):
    """Real-time telemetry data from the robot."""
    
    state: RobotState = Field(
        description="Current operational state of the robot"
    )
    battery: int = Field(
        ge=0,
        le=100,
        description="Battery level percentage (0-100)"
    )
    timestamp: datetime = Field(
        default_factory=datetime.utcnow,
        description="Timestamp of the telemetry reading"
    )

    class Config:
        json_encoders = {
            datetime: lambda v: v.isoformat() + "Z"
        }

