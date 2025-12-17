from pydantic import BaseModel
from typing import List, TYPE_CHECKING
if TYPE_CHECKING:
    from .section import Section


class Chapter(BaseModel):
    """
    Major division of Physical AI & Humanoid Robotics textbook content with title, content, and associated resources.
    """
    id: str
    title: str
    position: int
    sections: List['Section'] = []
    word_count: int = 0
    learning_objectives: List[str] = []
    # Physical AI specific attributes
    module_type: str  # e.g., "ROS 2", "Gazebo", "NVIDIA Isaac", "Vision-Language-Action"
    hardware_focus: List[str] = []  # e.g., ["simulation", "real-hardware", "cloud-deployment"]
    difficulty_level: str  # beginner, intermediate, advanced
    estimated_duration: str  # e.g., "2 weeks", "3 hours", "1 module"
    prerequisites: List[str] = []  # e.g., ["Python basics", "Linux command line"]
    # Personalization support
    personalized_content: dict = {}  # Content adapted for different user backgrounds
    # Translation support
    translated_titles: dict = {}  # {language_code: translated_title}