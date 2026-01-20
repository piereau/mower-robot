"""WebSocket endpoint for real-time robot telemetry."""

import asyncio
import json
import logging
from typing import Any, Dict, Set

from fastapi import APIRouter, WebSocket, WebSocketDisconnect

from ..control.service import get_command_service
from ..domain.models import RobotDriveCommand
from ..settings import settings
from ..simulation.telemetry import get_simulator


logger = logging.getLogger(__name__)
router = APIRouter()

# Connected WebSocket clients
connected_clients: Set[WebSocket] = set()


async def broadcast_telemetry() -> None:
    """Broadcast telemetry to all connected clients.
    
    This coroutine runs continuously, sending telemetry updates
    at the configured interval to all connected WebSocket clients.
    """
    simulator = get_simulator()
    
    while True:
        if connected_clients:
            # Advance simulation and get new telemetry
            telemetry = simulator.tick()
            message = telemetry.model_dump_json()
            
            # Broadcast to all connected clients
            disconnected = set()
            for client in connected_clients:
                try:
                    await client.send_text(message)
                except Exception as e:
                    logger.warning(f"Failed to send to client: {e}")
                    disconnected.add(client)
            
            # Remove disconnected clients
            connected_clients.difference_update(disconnected)
        
        await asyncio.sleep(settings.simulation_interval)


@router.websocket("/ws/robot")
async def robot_telemetry_ws(websocket: WebSocket) -> None:
    """WebSocket endpoint for real-time robot telemetry.
    
    Clients connect to this endpoint to receive continuous
    telemetry updates from the robot (or simulator in POC mode).
    
    Protocol:
    - Server pushes JSON telemetry messages at regular intervals
    - Client can send ping/pong for connection health
    - Connection closes on client disconnect or error
    """
    await websocket.accept()
    connected_clients.add(websocket)
    
    client_host = websocket.client.host if websocket.client else "unknown"
    logger.info(f"WebSocket client connected: {client_host}")
    
    # Send initial telemetry immediately
    simulator = get_simulator()
    initial_telemetry = simulator.get_telemetry()
    await websocket.send_text(initial_telemetry.model_dump_json())
    
    command_service = get_command_service()

    try:
        # Keep connection alive and handle any client messages
        while True:
            # Wait for client messages (ping/pong, close, etc.)
            data = await websocket.receive_text()
            logger.debug(f"Received from client: {data}")
            
            # Echo back pings for connection health
            if data == "ping":
                await websocket.send_text("pong")
                continue

            payload = _parse_json_message(data)
            if not payload:
                continue

            message_type = payload.get("type")
            if message_type == "control":
                _handle_control_message(payload, command_service)
    
    except WebSocketDisconnect:
        logger.info(f"WebSocket client disconnected: {client_host}")
    except Exception as e:
        logger.error(f"WebSocket error: {e}")
    finally:
        connected_clients.discard(websocket)
        command_service.stop()


def _parse_json_message(data: str) -> Dict[str, Any] | None:
    try:
        return json.loads(data)
    except json.JSONDecodeError:
        logger.debug("Ignoring non-JSON message")
        return None


def _handle_control_message(payload: Dict[str, Any], command_service) -> None:
    try:
        command = RobotDriveCommand(
            left=payload.get("left", 0.0),
            right=payload.get("right", 0.0),
        )
    except Exception as exc:
        logger.warning("Invalid control payload: %s", exc)
        return

    command_service.apply_drive_command(command)

