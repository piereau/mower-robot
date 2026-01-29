"""FastAPI application entrypoint."""

import asyncio
import logging
from contextlib import asynccontextmanager

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware

from .api.camera import router as camera_router
from .api.health import router as health_router
from .api.robot import router as robot_router
from .ws.robot import router as ws_router, broadcast_telemetry
from .settings import settings


# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
)
logger = logging.getLogger(__name__)


@asynccontextmanager
async def lifespan(app: FastAPI):
    """Application lifespan manager.
    
    Starts background tasks on startup and cleans up on shutdown.
    """
    logger.info("Starting RoboMower backend...")
    logger.info(f"Simulation interval: {settings.simulation_interval}s")
    logger.info(f"Using mock GPIO: {settings.use_mock_gpio}")
    
    # Start telemetry broadcast task
    broadcast_task = asyncio.create_task(broadcast_telemetry())
    logger.info("Telemetry broadcast task started")
    
    yield
    
    # Cleanup
    broadcast_task.cancel()
    try:
        await broadcast_task
    except asyncio.CancelledError:
        pass
    
    logger.info("RoboMower backend stopped")


app = FastAPI(
    title="RoboMower API",
    description="Backend API for the RoboMower autonomous lawn mower",
    version="0.1.0",
    lifespan=lifespan,
)

# Configure CORS for frontend development
app.add_middleware(
    CORSMiddleware,
    allow_origins=settings.cors_origins,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Include routers
app.include_router(health_router, tags=["Health"])
app.include_router(robot_router, tags=["Robot"])
app.include_router(camera_router, tags=["Camera"])
app.include_router(ws_router, tags=["WebSocket"])


@app.get("/")
async def root():
    """Root endpoint with API information."""
    return {
        "name": "RoboMower API",
        "version": "0.1.0",
        "docs": "/docs",
    }

