"""Raspberry Pi camera stream using libcamera-vid."""

import shutil
import subprocess
from typing import Iterator

from ..settings import settings
from .interfaces import CameraStream


def _extract_jpeg_frames(stream) -> Iterator[bytes]:
    buffer = b""
    while True:
        chunk = stream.read(4096)
        if not chunk:
            break
        buffer += chunk
        while True:
            start = buffer.find(b"\xff\xd8")
            if start == -1:
                break
            end = buffer.find(b"\xff\xd9", start + 2)
            if end == -1:
                break
            frame = buffer[start : end + 2]
            buffer = buffer[end + 2 :]
            yield frame


class LibCameraStream(CameraStream):
    """MJPEG stream from libcamera-vid."""

    def frames(self) -> Iterator[bytes]:
        cmd_name = shutil.which("libcamera-vid") or shutil.which("rpicam-vid")
        if not cmd_name:
            raise RuntimeError(
                "Camera binary not found. Install libcamera apps "
                "(libcamera-vid or rpicam-vid)."
            )

        cmd = [
            cmd_name,
            "--inline",
            "--codec",
            "mjpeg",
            "--width",
            str(settings.camera_width),
            "--height",
            str(settings.camera_height),
            "--framerate",
            str(settings.camera_fps),
        ]

        if settings.camera_hflip:
            cmd.append("--hflip")

        if settings.camera_vflip:
            cmd.append("--vflip")

        cmd.extend(["-t", "0", "-o", "-"])

        process = subprocess.Popen(  # nosec B603
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
        )

        if not process.stdout:
            return

        try:
            for frame in _extract_jpeg_frames(process.stdout):
                yield (
                    b"--frame\r\n"
                    b"Content-Type: image/jpeg\r\n\r\n"
                    + frame
                    + b"\r\n"
                )
        finally:
            process.terminate()
            process.wait(timeout=2)
