from pydantic import BaseModel
from typing import List, Optional


class UserPreferences(BaseModel):
    """
    User preferences for textbook generation and export.
    """
    id: str
    default_difficulty: str  # beginner, intermediate, advanced
    preferred_export_format: str = "pdf"
    custom_subjects: List[str] = []
    export_history: List[dict] = {}