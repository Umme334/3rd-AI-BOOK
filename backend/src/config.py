"""
Configuration settings for the textbook generation backend
"""

import os
from typing import Optional
from pydantic import BaseSettings
from dotenv import load_dotenv

# Load environment variables from .env file
load_dotenv()

class Settings(BaseSettings):
    # Application settings
    app_name: str = "AI-Native Textbook for Physical AI and Humanoid Robotics"
    app_version: str = "1.0.0"
    debug: bool = False

    # Database settings
    database_url: str = os.getenv("DATABASE_URL", "sqlite:///./textbook_generation.db")
    database_echo: bool = os.getenv("DATABASE_ECHO", "False").lower() == "true"

    # API settings
    api_host: str = os.getenv("API_HOST", "0.0.0.0")
    api_port: int = int(os.getenv("API_PORT", "8000"))
    api_workers: int = int(os.getenv("API_WORKERS", "1"))

    # Authentication settings
    jwt_secret_key: str = os.getenv("JWT_SECRET_KEY", "your-super-secret-key-change-in-production")
    jwt_algorithm: str = os.getenv("JWT_ALGORITHM", "HS256")
    access_token_expire_minutes: int = int(os.getenv("ACCESS_TOKEN_EXPIRE_MINUTES", "30"))

    # OpenAI settings
    openai_api_key: Optional[str] = os.getenv("OPENAI_API_KEY")
    openai_model: str = os.getenv("OPENAI_MODEL", "gpt-4")

    # Google Cloud settings (for translation)
    google_cloud_project_id: Optional[str] = os.getenv("GOOGLE_CLOUD_PROJECT_ID")
    google_cloud_region: str = os.getenv("GOOGLE_CLOUD_REGION", "us-central1")

    # Vector database settings
    vector_db_url: str = os.getenv("VECTOR_DB_URL", "http://localhost:6333")
    vector_db_collection_name: str = os.getenv("VECTOR_DB_COLLECTION_NAME", "textbooks")

    # CORS settings
    cors_origins: list = [
        "http://localhost",
        "http://localhost:3000",
        "http://localhost:3001",
        "http://127.0.0.1:3000",
        "http://127.0.0.1:8000",
    ]

    # Rate limiting
    rate_limit_requests: int = int(os.getenv("RATE_LIMIT_REQUESTS", "60"))
    rate_limit_window: int = int(os.getenv("RATE_LIMIT_WINDOW", "60"))

    # Logging
    log_level: str = os.getenv("LOG_LEVEL", "INFO")
    log_file: str = os.getenv("LOG_FILE", "logs/app.log")

    # Security
    allowed_hosts: list = ["localhost", "127.0.0.1", "[::1]"]
    secure_headers: bool = os.getenv("SECURE_HEADERS", "True").lower() == "true"

    class Config:
        env_file = ".env"
        case_sensitive = False

# Global settings instance
settings = Settings()

# Convenience function to get settings
def get_settings() -> Settings:
    return settings