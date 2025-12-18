from sqlalchemy import Column, Integer, String, Boolean, DateTime, Text
from sqlalchemy.dialects.postgresql import UUID
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.sql import func
import uuid

Base = declarative_base()


class User(Base):
    __tablename__ = "users"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    email = Column(String, unique=True, index=True, nullable=False)
    username = Column(String, unique=True, index=True, nullable=False)
    hashed_password = Column(String, nullable=False)
    full_name = Column(String, nullable=True)
    is_active = Column(Boolean, default=True)
    is_verified = Column(Boolean, default=False)
    created_at = Column(DateTime, default=func.now())
    updated_at = Column(DateTime, default=func.now(), onupdate=func.now())

    # User background information for personalization
    software_experience = Column(Integer, nullable=True)  # 1-5 scale
    hardware_experience = Column(Integer, nullable=True)  # 1-5 scale
    education_level = Column(String, nullable=True)  # e.g., "undergraduate", "graduate", "professional"
    field_of_study = Column(String, nullable=True)  # e.g., "computer science", "robotics", "engineering"
    programming_languages = Column(Text, nullable=True)  # JSON string of languages
    math_background = Column(String, nullable=True)  # e.g., "basic", "intermediate", "advanced"
    primary_goal = Column(Text, nullable=True)  # User's primary learning goal