"""Camera streaming interfaces."""

from abc import ABC, abstractmethod
from typing import Iterator


class CameraStream(ABC):
    """Abstract interface for camera streaming."""

    @abstractmethod
    def frames(self) -> Iterator[bytes]:
        """Yield multipart MJPEG frames."""
        raise NotImplementedError
