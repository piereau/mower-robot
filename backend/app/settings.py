"""Application settings and configuration."""

from pydantic_settings import BaseSettings


class Settings(BaseSettings):
    """Application configuration loaded from environment variables."""
    
    # Simulation settings
    simulation_interval: float = 2.0  # Seconds between telemetry broadcasts
    
    # Hardware settings
    use_mock_gpio: bool = True  # Use mock GPIO for development
    
    # Server settings
    host: str = "0.0.0.0"
    port: int = 8000
    
    # CORS settings (for frontend development)
    cors_origins: list[str] = ["http://localhost:5173", "http://localhost:3000"]
    
    class Config:
        env_file = ".env"
        env_file_encoding = "utf-8"


settings = Settings()

