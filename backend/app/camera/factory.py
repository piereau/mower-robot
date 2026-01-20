"""Factory for camera stream selection."""

from ..settings import settings
from .interfaces import CameraStream
from .libcamera_stream import LibCameraStream
from .mock_stream import MockCameraStream


def create_camera_stream() -> CameraStream:
    """Create camera stream based on settings."""
    if settings.use_mock_camera:
        return MockCameraStream()

    return LibCameraStream()
