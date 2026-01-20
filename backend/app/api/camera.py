"""Camera streaming endpoint."""

from fastapi import APIRouter
from fastapi.responses import StreamingResponse

from ..camera.factory import create_camera_stream

router = APIRouter()


@router.get("/camera/stream")
async def camera_stream() -> StreamingResponse:
    """Stream MJPEG frames from the camera."""
    stream = create_camera_stream()
    return StreamingResponse(
        stream.frames(),
        media_type="multipart/x-mixed-replace; boundary=frame",
    )
