from pydantic import BaseModel
from typing import List, Optional
from datetime import datetime
from enum import Enum
from .textbook_request import DifficultyLevel


class TextbookStatus(str, Enum):
    draft = "draft"
    generating = "generating"
    complete = "complete"
    failed = "failed"


class InteractiveElementResponse(BaseModel):
    """
    Response schema for interactive elements in textbooks.
    """
    id: str
    type: str  # quiz, summary, booster, exercise
    content: str
    position: int
    metadata: Optional[dict] = {}


class SectionResponse(BaseModel):
    """
    Response schema for sections in textbooks.
    """
    id: str
    title: str
    content: str
    position: int
    interactive_elements: List[InteractiveElementResponse] = []
    key_terms: List[str] = []


class ChapterResponse(BaseModel):
    """
    Response schema for chapters in textbooks.
    """
    id: str
    title: str
    position: int
    sections: List[SectionResponse] = []
    word_count: int = 0
    learning_objectives: List[str] = []


class TextbookResponse(BaseModel):
    """
    Response schema for textbook details.
    """
    id: str
    title: str
    subject: str
    difficulty: DifficultyLevel
    target_audience: Optional[str] = None
    chapters: List[ChapterResponse] = []
    metadata: Optional[dict] = {}
    export_formats: List[str] = []
    status: TextbookStatus
    created_at: datetime
    updated_at: datetime

    class Config:
        # Allow the enum to be passed as a string
        use_enum_values = True


class TextbookPreview(BaseModel):
    """
    Response schema for textbook preview.
    """
    id: str
    title: str
    preview_content: str
    chapters_count: int