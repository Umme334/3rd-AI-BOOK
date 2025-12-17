from pydantic import BaseModel
from typing import List, Optional
from datetime import datetime


class UserProfile(BaseModel):
    """
    User profile containing software and hardware background information for personalization.
    """
    id: str
    email: str
    name: str
    software_background: str  # e.g., "beginner", "intermediate", "advanced" in programming
    hardware_background: str  # e.g., "none", "basic", "advanced" in robotics/hardware
    programming_languages: List[str] = []
    hardware_experience: List[str] = []  # e.g., ["raspberry pi", "arduino", "jetson"]
    robotics_experience: str  # e.g., "none", "basic", "intermediate", "advanced"
    education_level: str  # e.g., "undergraduate", "graduate", "professional"
    primary_language: str = "en"
    preferences: dict = {}
    created_at: datetime = None
    updated_at: datetime = None

    def __init__(self, **data):
        super().__init__(**data)
        if self.created_at is None:
            self.created_at = datetime.now()
        self.updated_at = datetime.now()