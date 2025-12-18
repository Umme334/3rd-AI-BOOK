from abc import ABC, abstractmethod
from typing import Any, Dict, Optional
import asyncio
from dataclasses import dataclass


@dataclass
class SkillResult:
    """Result structure for skill execution"""
    success: bool
    data: Any
    message: str = ""
    metadata: Optional[Dict[str, Any]] = None


class BaseSkill(ABC):
    """Base class for all skills in the system"""

    def __init__(self, skill_id: str, name: str, description: str):
        self.skill_id = skill_id
        self.name = name
        self.description = description
        self.is_available = True

    @abstractmethod
    async def execute(self, **kwargs) -> SkillResult:
        """Execute the skill with given parameters"""
        pass

    async def validate_inputs(self, **kwargs) -> bool:
        """Validate input parameters before execution"""
        return True

    def get_info(self) -> Dict[str, Any]:
        """Get skill information"""
        return {
            "id": self.skill_id,
            "name": self.name,
            "description": self.description,
            "available": self.is_available
        }