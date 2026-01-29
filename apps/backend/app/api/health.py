"""Health check endpoint."""

from fastapi import APIRouter
from pydantic import BaseModel


router = APIRouter()


class HealthResponse(BaseModel):
    """Health check response."""
    status: str


@router.get("/health", response_model=HealthResponse)
async def health_check() -> HealthResponse:
    """Check if the server is running.
    
    Returns:
        Simple status response indicating the server is healthy
    """
    return HealthResponse(status="ok")

