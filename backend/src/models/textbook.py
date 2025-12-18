from pydantic import BaseModel
from typing import List, Optional, Any
from datetime import datetime
from enum import Enum
from __future__ import annotations  # Enable postponed evaluation of annotations


class TextbookStatus(str, Enum):
    draft = "draft"
    generating = "generating"
    complete = "complete"
    failed = "failed"


class Textbook(BaseModel):
    """
    Represents an educational textbook with chapters, sections, and interactive elements for Physical AI & Humanoid Robotics.
    """
    id: str
    title: str
    subject: str  # Should be "Physical AI and Humanoid Robotics" or related
    difficulty: str  # beginner, intermediate, advanced
    target_audience: Optional[str] = None
    chapters: List['Chapter'] = []
    metadata: Optional[dict] = {}
    export_formats: List[str] = []
    status: TextbookStatus = TextbookStatus.draft
    created_at: datetime = None
    updated_at: datetime = None
    chapter_count: int = 5
    # Physical AI specific attributes
    course_module: Optional[str] = None  # e.g., "ROS 2", "Gazebo", "NVIDIA Isaac", "Vision-Language-Action"
    hardware_requirements: List[str] = []  # e.g., ["RTX 4080", "Jetson Orin", "RealSense D435i"]
    learning_outcomes: List[str] = []
    # RAG and personalization support
    rag_indexed: bool = False
    rag_index_id: Optional[str] = None  # ID of the RAG index in vector store
    rag_last_indexed: Optional[datetime] = None  # When the textbook was last indexed for RAG
    personalization_enabled: bool = False
    personalization_settings: Optional[dict] = {}  # Personalization settings per user
    personalized_content_cache: Optional[dict] = {}  # Cache of personalized content
    # Translation support
    available_translations: List[str] = ["en"]  # List of available language codes

    def __init__(self, **data):
        super().__init__(**data)
        if self.created_at is None:
            self.created_at = datetime.now()
        self.updated_at = datetime.now()
        if not self.export_formats:
            self.export_formats = ["pdf", "html", "markdown", "docusaurus"]  # Added docusaurus export
        if not self.hardware_requirements:
            self.hardware_requirements = []
        if not self.learning_outcomes:
            self.learning_outcomes = []
        if not self.available_translations:
            self.available_translations = ["en"]

    @classmethod
    def model_rebuild(cls, **kwargs):
        # Rebuild the model to resolve forward references
        super().model_rebuild(**kwargs)