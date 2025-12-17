from pydantic import BaseModel
from typing import Optional
from datetime import datetime


class TranslationCache(BaseModel):
    """
    Caches translated content to improve performance for Urdu translation feature.
    """
    id: str
    original_content_id: str  # ID of the original content (chapter, section, etc.)
    original_content: str
    translated_content: str
    source_language: str = "en"
    target_language: str = "ur"
    content_type: str  # "chapter", "section", "interactive_element", etc.
    created_at: datetime = None
    updated_at: datetime = None
    expires_at: Optional[datetime] = None

    def __init__(self, **data):
        super().__init__(**data)
        if self.created_at is None:
            self.created_at = datetime.now()
        self.updated_at = datetime.now()