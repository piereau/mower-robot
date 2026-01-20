"""Mock camera stream for development without hardware."""

import base64
import time
from typing import Iterator

from .interfaces import CameraStream

# 1x1 white JPEG (base64)
_PLACEHOLDER_JPEG = base64.b64decode(
    b"/9j/4AAQSkZJRgABAQAAAQABAAD/2wCEAAkGBxAQEBUQEBAVFRUVFRUVFRUVFRUVFRUWFhUX"
    b"FhUYHSggGBolGxUVITEhJSkrLi4uFx8zODMtNygtLisBCgoKDg0OFxAQGy0lHyUtLS0tLS0t"
    b"LS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLf/AABEIAAEAAQMBIgACEQED"
    b"EQH/xAAbAAACAgMBAAAAAAAAAAAAAAAEBQMGAAIHB//EADcQAAEDAgMFBgQEBwAAAAAAAAEA"
    b"AgMEBREABhIhMQcTQVFhInGBMpGxFDJScpLR8COB0f/EABgBAQEBAQEAAAAAAAAAAAAAAAAB"
    b"AgME/8QAHBEBAQEAAwEBAQAAAAAAAAAAAAERAgMhEjFR/9oADAMBAAIRAxEAPwDk2kD7KJ4E"
    b"3NQv9b4D0cB0mQ7c0e2tYvQh3J1m2S3bQb0G7i9bq3xF3e1bD4S1eG1w5sJx9i8j5Q2H4g7VZ"
    b"gkZ8Jw5H8Xh5qv7bq2l9tW2P//Z"
)


class MockCameraStream(CameraStream):
    """Mock camera stream that yields a static frame."""

    def frames(self) -> Iterator[bytes]:
        while True:
            yield (
                b"--frame\r\n"
                b"Content-Type: image/jpeg\r\n\r\n"
                + _PLACEHOLDER_JPEG
                + b"\r\n"
            )
            time.sleep(0.2)
