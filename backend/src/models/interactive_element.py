from pydantic import BaseModel
from typing import Optional, List


class InteractiveElement(BaseModel):
    """
    Components like quizzes, summaries, and learning boosters embedded in Physical AI & Humanoid Robotics content.
    """
    id: str
    type: str  # quiz, summary, booster, exercise, simulation, code-challenge
    content: str
    position: int
    metadata: Optional[dict] = {}
    # Physical AI specific attributes
    hardware_relevance: List[str] = []  # e.g., ["simulation", "real-robot", "cloud"]
    difficulty_level: str  # beginner, intermediate, advanced
    learning_objective: str  # Specific learning objective for this element
    # Personalization support
    personalized_variants: dict = {}  # Different versions based on user background
    # Translation support
    translated_content: dict = {}  # {language_code: translated_content}