from typing import Optional, AsyncGenerator
from pydantic import BaseSettings
import os
from sqlalchemy.ext.asyncio import create_async_engine, AsyncSession
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.orm import sessionmaker


class DatabaseSettings(BaseSettings):
    DATABASE_URL: str = os.getenv("DATABASE_URL", "sqlite+aiosqlite:///./textbook_app.db")

    class Config:
        env_file = ".env"


# Create async database engine for development (using SQLite by default)
database_url = os.getenv("DATABASE_URL", "sqlite+aiosqlite:///./textbook_app.db")
if not database_url or database_url == "":
    database_url = "sqlite+aiosqlite:///./textbook_app.db"

engine = create_async_engine(
    database_url,
    pool_pre_ping=True,
    pool_recycle=300,
    echo=False  # Set to True for debugging
)

# Create async session factory
AsyncSessionLocal = sessionmaker(
    engine,
    class_=AsyncSession,
    expire_on_commit=False
)

# Create base class for models
Base = declarative_base()


async def get_db() -> AsyncGenerator[AsyncSession, None]:
    """Async dependency for getting database session"""
    async with AsyncSessionLocal() as session:
        try:
            yield session
        finally:
            await session.close()