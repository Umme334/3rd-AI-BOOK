from typing import List, Dict, Any
import uuid

from ...models.interactive_element import InteractiveElement
from ...services.openai_client import OpenAIClient
from ...services.content_generation.difficulty_service import DifficultyService
from ..base_service import BaseService


class InteractiveElementService(BaseService):
    """
    Service for generating interactive elements like quizzes, summaries, and learning boosters.
    """

    def __init__(self):
        super().__init__()
        self.openai_client = OpenAIClient()
        self.difficulty_service = DifficultyService()

    def generate_interactive_elements(
        self,
        content_context: str,
        difficulty: str,
        element_types: List[str] = None,
        count: int = 1
    ) -> List[InteractiveElement]:
        """
        Generate interactive elements based on content context and difficulty.

        Args:
            content_context: The content to generate interactive elements for
            difficulty: Difficulty level (beginner, intermediate, advanced)
            element_types: Types of interactive elements to generate (quiz, summary, booster, exercise)
            count: Number of elements to generate per type

        Returns:
            List of InteractiveElement objects
        """
        if element_types is None:
            element_types = ["quiz", "summary"]

        elements = []
        position = 1

        for element_type in element_types:
            for _ in range(count):
                element_content = self.openai_client.generate_interactive_element(
                    element_type=element_type,
                    content_context=content_context,
                    difficulty=difficulty
                )

                element = InteractiveElement(
                    id=f"ie-{element_type}-{uuid.uuid4().hex[:8]}",
                    type=element_type,
                    content=element_content,
                    position=position,
                    metadata={
                        "difficulty": difficulty,
                        "generated_at": self.created_at.isoformat()
                    }
                )

                elements.append(element)
                position += 1

        return elements

    def generate_quiz_for_content(
        self,
        content_context: str,
        difficulty: str,
        num_questions: int = 5
    ) -> InteractiveElement:
        """
        Generate a quiz specifically for the given content.

        Args:
            content_context: The content to generate a quiz for
            difficulty: Difficulty level
            num_questions: Number of questions to include

        Returns:
            InteractiveElement object containing the quiz
        """
        # Customize content for difficulty
        customized_context = self.difficulty_service.customize_content_for_difficulty(
            content_context, difficulty
        )

        # Generate quiz content
        quiz_content = self.openai_client.generate_interactive_element(
            element_type="quiz",
            content_context=customized_context,
            difficulty=difficulty
        )

        element = InteractiveElement(
            id=f"ie-quiz-{uuid.uuid4().hex[:8]}",
            type="quiz",
            content=quiz_content,
            position=1,
            metadata={
                "difficulty": difficulty,
                "question_count": num_questions,
                "generated_at": self.created_at.isoformat()
            }
        )

        return element

    def generate_summary_for_content(
        self,
        content_context: str,
        difficulty: str
    ) -> InteractiveElement:
        """
        Generate a summary for the given content.

        Args:
            content_context: The content to generate a summary for
            difficulty: Difficulty level

        Returns:
            InteractiveElement object containing the summary
        """
        # Customize content for difficulty
        customized_context = self.difficulty_service.customize_content_for_difficulty(
            content_context, difficulty
        )

        # Generate summary content
        summary_content = self.openai_client.generate_interactive_element(
            element_type="summary",
            content_context=customized_context,
            difficulty=difficulty
        )

        element = InteractiveElement(
            id=f"ie-summary-{uuid.uuid4().hex[:8]}",
            type="summary",
            content=summary_content,
            position=1,
            metadata={
                "difficulty": difficulty,
                "generated_at": self.created_at.isoformat()
            }
        )

        return element

    def generate_learning_booster(
        self,
        content_context: str,
        difficulty: str,
        booster_type: str = "exercise"
    ) -> InteractiveElement:
        """
        Generate a learning booster (exercise, activity, etc.) for the given content.

        Args:
            content_context: The content to generate a booster for
            difficulty: Difficulty level
            booster_type: Type of booster (exercise, activity, discussion, etc.)

        Returns:
            InteractiveElement object containing the learning booster
        """
        # Customize content for difficulty
        customized_context = self.difficulty_service.customize_content_for_difficulty(
            content_context, difficulty
        )

        # Generate booster content
        booster_content = self.openai_client.generate_interactive_element(
            element_type=booster_type,
            content_context=customized_context,
            difficulty=difficulty
        )

        element = InteractiveElement(
            id=f"ie-{booster_type}-{uuid.uuid4().hex[:8]}",
            type=booster_type,
            content=booster_content,
            position=1,
            metadata={
                "difficulty": difficulty,
                "booster_type": booster_type,
                "generated_at": self.created_at.isoformat()
            }
        )

        return element

    def enhance_content_with_interactives(
        self,
        content: str,
        difficulty: str,
        include_types: List[str] = None
    ) -> Dict[str, Any]:
        """
        Enhance content with interactive elements.

        Args:
            content: Original content to enhance
            difficulty: Difficulty level
            include_types: Types of interactive elements to include

        Returns:
            Dictionary with enhanced content and interactive elements
        """
        if include_types is None:
            include_types = ["quiz", "summary"]

        interactives = self.generate_interactive_elements(
            content_context=content,
            difficulty=difficulty,
            element_types=include_types
        )

        return {
            "original_content": content,
            "interactive_elements": interactives,
            "enhanced_content": self._integrate_interactives(content, interactives)
        }

    def _integrate_interactives(self, content: str, interactives: List[InteractiveElement]) -> str:
        """
        Integrate interactive elements into the content at appropriate positions.

        Args:
            content: Original content
            interactives: List of interactive elements to integrate

        Returns:
            Content with integrated interactive elements
        """
        # For now, simply append interactive elements to the content
        # In a real implementation, this would place them at more appropriate positions
        enhanced_content = content

        for interactive in interactives:
            enhanced_content += f"\n\n[{interactive.type.upper()}]: {interactive.content}"

        return enhanced_content