from pydantic import BaseModel
from typing import List, Any
from __future__ import annotations  # Enable postponed evaluation of annotations


class Section(BaseModel):
    """
    Subdivision of Physical AI & Humanoid Robotics chapters with focused topic coverage and learning objectives.
    """
    id: str
    title: str
    content: str
    position: int
    interactive_elements: List[Any] = []
    key_terms: List[str] = []
    # Physical AI specific attributes
    topic_category: str  # e.g., "theoretical", "practical", "simulation", "hardware"
    hardware_requirements: List[str] = []  # Specific hardware needed for this section
    code_examples: List[str] = []  # List of code example IDs or content
    # Personalization support
    personalized_content: dict = {}  # Content adapted for different user backgrounds
    difficulty_adjustments: dict = {}  # Difficulty adjustments based on user background
    # Translation support
    translated_content: dict = {}  # {language_code: translated_content}

    @classmethod
    def model_rebuild(cls, **kwargs):
        # Rebuild the model to resolve forward references
        super().model_rebuild(**kwargs)