from typing import Dict, List, Optional, Any
from .base_skill import BaseSkill, SkillResult


class SkillsManager:
    """Manages registration, discovery, and execution of skills"""

    def __init__(self):
        self.skills: Dict[str, BaseSkill] = {}
        self._initialize_default_skills()

    def _initialize_default_skills(self):
        """Initialize default skills for the RAG system"""
        from .retrieval_skill import TextRetrievalSkill, ContentIndexingSkill
        from .generation_skill import TextGenerationSkill, TextSummarizationSkill, QuestionAnsweringSkill
        from .content_skill import ContentChunkingSkill, ContentAnalysisSkill, TextExtractionSkill

        default_skills = [
            TextRetrievalSkill(),
            ContentIndexingSkill(),
            TextGenerationSkill(),
            TextSummarizationSkill(),
            QuestionAnsweringSkill(),
            ContentChunkingSkill(),
            ContentAnalysisSkill(),
            TextExtractionSkill()
        ]

        for skill in default_skills:
            self.register_skill(skill)

    def register_skill(self, skill: BaseSkill) -> bool:
        """Register a new skill"""
        try:
            if skill.skill_id in self.skills:
                print(f"Warning: Skill {skill.skill_id} already exists, overwriting")

            self.skills[skill.skill_id] = skill
            return True
        except Exception as e:
            print(f"Error registering skill {skill.skill_id}: {str(e)}")
            return False

    def unregister_skill(self, skill_id: str) -> bool:
        """Unregister a skill"""
        if skill_id in self.skills:
            del self.skills[skill_id]
            return True
        return False

    async def execute_skill(self, skill_id: str, **kwargs) -> Optional[SkillResult]:
        """Execute a skill by ID"""
        if skill_id not in self.skills:
            return SkillResult(
                success=False,
                data=None,
                message=f"Skill {skill_id} not found"
            )

        skill = self.skills[skill_id]

        # Validate inputs before execution
        try:
            is_valid = await skill.validate_inputs(**kwargs)
            if not is_valid:
                return SkillResult(
                    success=False,
                    data=None,
                    message=f"Invalid inputs for skill {skill_id}"
                )
        except Exception as e:
            return SkillResult(
                success=False,
                data=None,
                message=f"Error validating inputs for skill {skill_id}: {str(e)}"
            )

        # Execute the skill
        try:
            result = await skill.execute(**kwargs)
            return result
        except Exception as e:
            return SkillResult(
                success=False,
                data=None,
                message=f"Error executing skill {skill_id}: {str(e)}"
            )

    def get_skill(self, skill_id: str) -> Optional[BaseSkill]:
        """Get a skill by ID"""
        return self.skills.get(skill_id)

    def get_all_skills(self) -> List[BaseSkill]:
        """Get all registered skills"""
        return list(self.skills.values())

    def get_skill_info(self, skill_id: str) -> Optional[Dict[str, Any]]:
        """Get information about a specific skill"""
        skill = self.get_skill(skill_id)
        if skill:
            return skill.get_info()
        return None

    def get_available_skills(self) -> List[Dict[str, Any]]:
        """Get information about all available skills"""
        return [skill.get_info() for skill in self.skills.values() if skill.is_available]

    def skill_exists(self, skill_id: str) -> bool:
        """Check if a skill exists"""
        return skill_id in self.skills

    async def batch_execute(self, skill_executions: List[Dict[str, Any]]) -> List[SkillResult]:
        """Execute multiple skills in batch"""
        results = []
        for execution in skill_executions:
            skill_id = execution.get("skill_id")
            if skill_id:
                kwargs = execution.get("kwargs", {})
                result = await self.execute_skill(skill_id, **kwargs)
                results.append(result)
            else:
                results.append(SkillResult(
                    success=False,
                    data=None,
                    message="Missing skill_id in execution request"
                ))
        return results

    async def execute_skill_chain(self, skill_chain: List[Dict[str, Any]]) -> List[SkillResult]:
        """Execute skills in sequence, passing output of one as input to the next"""
        results = []
        previous_result = None

        for i, execution in enumerate(skill_chain):
            skill_id = execution.get("skill_id")
            if not skill_id:
                results.append(SkillResult(
                    success=False,
                    data=None,
                    message="Missing skill_id in chain execution"
                ))
                continue

            kwargs = execution.get("kwargs", {}).copy()

            # If this is not the first skill and previous result should be used
            if i > 0 and previous_result and previous_result.success:
                # Pass previous result as context or data to the next skill
                # This can be customized based on specific requirements
                if "context" not in kwargs and "data" not in kwargs:
                    kwargs["context"] = previous_result.data if previous_result.data else previous_result.message

            result = await self.execute_skill(skill_id, **kwargs)
            results.append(result)
            previous_result = result

            # If a skill fails, we might want to stop the chain (configurable behavior)
            if not result.success and execution.get("stop_on_failure", True):
                break

        return results