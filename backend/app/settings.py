"""Application settings and configuration."""

from pydantic_settings import BaseSettings


class Settings(BaseSettings):
    """Application configuration loaded from environment variables."""
    
    # Simulation settings
    simulation_interval: float = 2.0  # Seconds between telemetry broadcasts
    
    # Hardware settings
    use_mock_gpio: bool = True  # Use mock GPIO for development

    # Motor control settings
    use_mock_motor_controller: bool = True  # Use mock motor controller by default
    serial_port: str = "/dev/ttyUSB0"
    serial_baud: int = 115200

    # Camera settings
    use_mock_camera: bool = True  # Use mock camera stream by default
    camera_width: int = 640
    camera_height: int = 480
    camera_fps: int = 15
    
    # Server settings
    host: str = "0.0.0.0"
    port: int = 8000
    
    # CORS settings (for frontend development)
    cors_origins: list[str] = ["http://localhost:5173", "http://localhost:3000"]
    
    class Config:
        env_file = ".env"
        env_file_encoding = "utf-8"


settings = Settings()

