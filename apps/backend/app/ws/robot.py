"""WebSocket endpoint for real-time robot telemetry and control."""

import asyncio
import json
import logging
from typing import Any, Dict, Set

from fastapi import APIRouter, WebSocket, WebSocketDisconnect

from ..ros_bridge import get_ros_bridge_client
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
    bridge = get_ros_bridge_client()
    
    # Register event-driven callbacks
    # These will be called by the bridge client when new data arrives
    bridge.on_map(_broadcast_map)
    bridge.on_scan(_broadcast_scan)
    bridge.on_pose(_broadcast_pose)

    logger.info("Broadcast telemetry task started")
    last_log_time = 0.0
    
    try:
        while True:
            # Log periodic heartbeat (every 5 seconds)
            import time
            current_time = time.time()
            if current_time - last_log_time > 60.0:
                logger.debug(f"Broadcast loop alive. Connected clients: {len(connected_clients)}")
                last_log_time = current_time

            if connected_clients:
                # Advance simulation and get new telemetry
                telemetry = simulator.tick()
                
                # Merge bridge status if connected
                # Use mode='json' to ensure datetime objects are serialized to strings
                telemetry_dict = telemetry.model_dump(mode='json')
                
                # Log bridge status occasionally
                if current_time - last_log_time > 5.0 and bridge.connected:
                     logger.info(f"Bridge connected: {bridge.connected}")

                if bridge.connected:
                    telemetry_dict['bridge_connected'] = True
                    telemetry_dict['bridge_status'] = {
                        'serial_connected': bridge.status.connected,
                        'watchdog': bridge.status.watchdog,
                        'estop': bridge.status.estop,
                        'battery_mv': bridge.status.battery_mv,
                        'linear_vel': bridge.status.linear_vel,
                        'angular_vel': bridge.status.angular_vel,
                    }
                else:
                    telemetry_dict['bridge_connected'] = False
                
                message = json.dumps(telemetry_dict)
                
                # Broadcast status to all connected clients
                await _broadcast_to_all(message)
            
            await asyncio.sleep(settings.simulation_interval)
            
    except Exception as e:
        logger.error(f"Broadcast task crashed: {e}", exc_info=True)
        raise e


async def _broadcast_map(msg: dict) -> None:
    """Broadcast map update to all clients."""
    await _broadcast_to_all(json.dumps(msg))


async def _broadcast_scan(msg: dict) -> None:
    """Broadcast scan update to all clients."""
    await _broadcast_to_all(json.dumps(msg))


async def _broadcast_pose(msg: dict) -> None:
    """Broadcast pose update to all clients."""
    await _broadcast_to_all(json.dumps(msg))


async def _broadcast_to_all(message: str) -> None:
    """Helper to send a message to all connected clients."""
    if not connected_clients:
        return
        
    disconnected = set()
    for client in connected_clients:
        try:
            await client.send_text(message)
        except Exception as e:
            logger.warning(f"Failed to send to client: {e}")
            disconnected.add(client)
    
    if disconnected:
        connected_clients.difference_update(disconnected)


@router.websocket("/ws/robot")
async def robot_telemetry_ws(websocket: WebSocket) -> None:
    """WebSocket endpoint for real-time robot telemetry and control.
    
    Clients connect to this endpoint to receive continuous
    telemetry updates and send control commands.
    
    Protocol:
    - Server pushes JSON telemetry messages at regular intervals
    - Client can send control messages: {"type": "control", "x": 0.5, "y": 0.3}
    - Client can send ping/pong for connection health
    - Connection closes on client disconnect or error
    """
    await websocket.accept()
    connected_clients.add(websocket)
    
    client_host = websocket.client.host if websocket.client else "unknown"
    logger.info(f"WebSocket client connected: {client_host}")
    
    # Get bridge client for control commands
    bridge = get_ros_bridge_client()
    
    # Attempt to connect to ROS 2 bridge (non-blocking)
    asyncio.create_task(_ensure_bridge_connected(bridge))
    
    # Send initial telemetry immediately
    simulator = get_simulator()
    initial_telemetry = simulator.get_telemetry()
    await websocket.send_text(initial_telemetry.model_dump_json())

    try:
        # Keep connection alive and handle any client messages
        while True:
            # Wait for client messages (ping/pong, control, etc.)
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
                await _handle_control_message(payload, bridge)
            elif message_type == "estop":
                await bridge.send_estop()
            elif message_type == "release_estop":
                await bridge.send_release_estop()
    
    except WebSocketDisconnect:
        logger.info(f"WebSocket client disconnected: {client_host}")
    except Exception as e:
        logger.error(f"WebSocket error: {e}")
    finally:
        connected_clients.discard(websocket)
        # Send zero velocity when client disconnects
        await bridge.send_velocity(0, 0)


async def _ensure_bridge_connected(bridge) -> None:
    """Attempt to connect to ROS 2 bridge in background.
    
    Will retry connection if not already connected.
    Logs the current bridge state for debugging.
    """
    logger.debug(f"Checking bridge connection: connected={bridge.connected}")
    
    if bridge.connected:
        logger.debug("Bridge already connected")
        return
        
    # Try to connect
    logger.info("Attempting to connect to ROS 2 bridge...")
    connected = await bridge.connect()
    
    if connected:
        logger.info("ROS 2 bridge connected successfully")
    else:
        logger.warning("ROS 2 bridge not available - control commands will fail gracefully")


def _parse_json_message(data: str) -> Dict[str, Any] | None:
    try:
        return json.loads(data)
    except json.JSONDecodeError:
        logger.debug("Ignoring non-JSON message")
        return None


async def _handle_control_message(payload: Dict[str, Any], bridge) -> None:
    """Handle control message from dashboard.
    
    Expects normalized joystick values:
    - x: left/right (-1 to 1), positive = right
    - y: forward/back (-1 to 1), positive = forward
    """
    try:
        x = float(payload.get("x", 0.0))
        y = float(payload.get("y", 0.0))
        
        # Clamp values to valid range
        x = max(-1.0, min(1.0, x))
        y = max(-1.0, min(1.0, y))
        
        # Send to ROS 2 bridge (graceful failure if not connected)
        success = await bridge.send_velocity(x, y)
        if not success:
            logger.debug("Control command not sent (bridge not connected)")
            
    except Exception as exc:
        logger.warning("Invalid control payload: %s", exc)
