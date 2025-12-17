from pydantic import BaseModel
from typing import Optional, List
from enum import Enum


class DifficultyLevel(str, Enum):
    beginner = "beginner"
    intermediate = "intermediate"
    advanced = "advanced"


class TextbookCreateRequest(BaseModel):
    """
    Request schema for creating a new Physical AI and Humanoid Robotics textbook.
    """
    title: str
    subject: str
    difficulty: DifficultyLevel
    target_audience: Optional[str] = None
    chapter_count: int = 5
    include_quizzes: bool = True
    include_summaries: bool = True
    # Physical AI specific attributes
    course_module: Optional[str] = None  # e.g., "ROS 2", "Gazebo", "NVIDIA Isaac", "Vision-Language-Action"
    hardware_requirements: Optional[List[str]] = None
    enable_personalization: bool = False
    enable_translation: bool = False

    class Config:
        # Allow the enum to be passed as a string
        use_enum_values = True

    def __init__(self, **data):
        super().__init__(**data)
        # Validate chapter count range
        if self.chapter_count < 1 or self.chapter_count > 20:
            raise ValueError("Chapter count must be between 1 and 20")


class TextbookStructureRequest(BaseModel):
    """
    Request schema for customizing textbook structure.
    """
    textbook_id: str
    chapter_organization: Optional[dict] = None  # Custom organization structure
    content_types: List[str] = ["text", "quizzes"]  # e.g., ["text", "images", "quizzes"]
    custom_learning_objectives: Optional[List[str]] = None
    exclude_chapters: List[int] = []  # Positions of chapters to exclude
    # Physical AI specific attributes
    module_type: Optional[str] = None  # e.g., "ROS 2", "Gazebo", "NVIDIA Isaac", "Vision-Language-Action"
    hardware_focus: Optional[List[str]] = None  # e.g., ["simulation", "real-hardware", "cloud-deployment"]
    enable_personalization: bool = False
    enable_translation: bool = False