from typing import Dict, Any, List, Optional
from datetime import datetime

from ...models.textbook import Textbook
from ...models.chapter import Chapter
from ...services.textbook_service import TextbookService
from ...utils.storage import FileStorage
from ...exceptions import TextbookNotFoundError, ValidationError
from ..base_service import BaseService
from ...schemas.textbook_request import TextbookStructureRequest


class TextbookStructureService(BaseService):
    """
    Service for managing textbook structure customization.
    """

    def __init__(self):
        super().__init__()
        self.textbook_service = TextbookService()
        self.storage = FileStorage()

    def customize_textbook_structure(self, request: TextbookStructureRequest) -> Textbook:
        """
        Customize the structure of an existing textbook.

        Args:
            request: Structure customization request

        Returns:
            Updated textbook with customized structure
        """
        # Validate the request
        self._validate_structure_request(request)

        # Get the existing textbook
        textbook = self.textbook_service.get_textbook(request.textbook_id)

        # Apply customizations
        if request.chapter_organization:
            textbook = self._reorganize_chapters(textbook, request.chapter_organization)

        if request.content_types:
            textbook = self._customize_content_types(textbook, request.content_types)

        if request.custom_learning_objectives:
            textbook = self._add_custom_learning_objectives(textbook, request.custom_learning_objectives)

        if request.exclude_chapters:
            textbook = self._exclude_chapters(textbook, request.exclude_chapters)

        # Update the textbook in storage
        textbook.updated_at = datetime.now()
        textbook_data = textbook.model_dump()
        self.storage.save_textbook(request.textbook_id, textbook_data)

        return textbook

    def _validate_structure_request(self, request: TextbookStructureRequest) -> None:
        """
        Validate the structure customization request.

        Args:
            request: Structure customization request to validate

        Raises:
            ValidationError: If the request is invalid
        """
        if not request.textbook_id or len(request.textbook_id.strip()) == 0:
            raise ValidationError("Textbook ID is required and cannot be empty")

        # Validate content types if provided
        if request.content_types:
            valid_content_types = ['text', 'images', 'quizzes', 'videos', 'exercises', 'summaries']
            for content_type in request.content_types:
                if content_type not in valid_content_types:
                    raise ValidationError(f"Invalid content type: {content_type}. Valid types are: {', '.join(valid_content_types)}")

        # Validate excluded chapters if provided
        if request.exclude_chapters:
            for chapter_pos in request.exclude_chapters:
                if not isinstance(chapter_pos, int) or chapter_pos < 1:
                    raise ValidationError(f"Excluded chapter positions must be positive integers, got: {chapter_pos}")

    def _reorganize_chapters(self, textbook: Textbook, organization: Dict[str, Any]) -> Textbook:
        """
        Reorganize chapters based on the provided organization structure.

        Args:
            textbook: Original textbook
            organization: Organization structure

        Returns:
            Textbook with reorganized chapters
        """
        # For now, just update the order of existing chapters
        # In a real implementation, this would involve more complex restructuring
        if 'order' in organization:
            new_chapter_order = organization['order']
            ordered_chapters = []

            for position in new_chapter_order:
                if 1 <= position <= len(textbook.chapters):
                    # Adjust for 0-based indexing
                    ordered_chapters.append(textbook.chapters[position - 1])

            textbook.chapters = ordered_chapters

            # Update positions
            for i, chapter in enumerate(ordered_chapters):
                chapter.position = i + 1

        # Add custom chapter information if provided
        if 'custom_titles' in organization:
            custom_titles = organization['custom_titles']
            for i, title in enumerate(custom_titles):
                if i < len(textbook.chapters):
                    textbook.chapters[i].title = title

        return textbook

    def _customize_content_types(self, textbook: Textbook, content_types: List[str]) -> Textbook:
        """
        Customize the content types included in the textbook.

        Args:
            textbook: Original textbook
            content_types: List of content types to include

        Returns:
            Textbook with customized content types
        """
        # This would modify which interactive elements are included
        # For now, just store the content types in metadata
        if not textbook.metadata:
            textbook.metadata = {}
        textbook.metadata['content_types'] = content_types

        return textbook

    def _add_custom_learning_objectives(self, textbook: Textbook, objectives: List[str]) -> Textbook:
        """
        Add custom learning objectives to the textbook.

        Args:
            textbook: Original textbook
            objectives: List of custom learning objectives

        Returns:
            Textbook with added learning objectives
        """
        # For now, add to the textbook metadata
        if not textbook.metadata:
            textbook.metadata = {}
        textbook.metadata['custom_learning_objectives'] = objectives

        return textbook

    def _exclude_chapters(self, textbook: Textbook, exclude_positions: List[int]) -> Textbook:
        """
        Exclude specified chapters from the textbook.

        Args:
            textbook: Original textbook
            exclude_positions: List of chapter positions to exclude

        Returns:
            Textbook with specified chapters excluded
        """
        # Filter out chapters at the specified positions
        # Note: positions are 1-based, so adjust for 0-based indexing
        filtered_chapters = []
        for i, chapter in enumerate(textbook.chapters):
            if (i + 1) not in exclude_positions:  # +1 to convert to 1-based
                filtered_chapters.append(chapter)

        # Update positions to be sequential again
        for i, chapter in enumerate(filtered_chapters):
            chapter.position = i + 1

        textbook.chapters = filtered_chapters

        return textbook

    def get_structure_options(self, subject: str, difficulty: str) -> Dict[str, Any]:
        """
        Get available structure options for a given subject and difficulty.

        Args:
            subject: Subject area
            difficulty: Difficulty level

        Returns:
            Dictionary with available structure options
        """
        # Provide default structure options based on subject and difficulty
        structure_options = {
            "chapter_organization": {
                "default": ["introduction", "fundamentals", "applications", "advanced_topics", "conclusion"],
                "alternative": ["theoretical", "practical", "case_studies", "review"]
            },
            "content_types": ["text", "quizzes", "summaries"],
            "learning_objective_templates": [
                f"Understand the fundamental concepts of {subject}",
                f"Apply {subject} principles to practical problems",
                f"Evaluate different approaches in {subject}"
            ],
            "recommended_chapter_count": self._get_recommended_chapter_count(difficulty)
        }

        if difficulty == "beginner":
            structure_options["content_types"].extend(["visual_aids", "simple_exercises"])
        elif difficulty == "intermediate":
            structure_options["content_types"].extend(["problem_sets", "examples"])
        elif difficulty == "advanced":
            structure_options["content_types"].extend(["research_papers", "critical_analysis"])

        return structure_options

    def _get_recommended_chapter_count(self, difficulty: str) -> int:
        """
        Get recommended chapter count based on difficulty.

        Args:
            difficulty: Difficulty level

        Returns:
            Recommended number of chapters
        """
        difficulty_to_chapters = {
            "beginner": 6,
            "intermediate": 8,
            "advanced": 10
        }
        return difficulty_to_chapters.get(difficulty, 5)