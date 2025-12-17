from typing import Dict, Any, List
from datetime import datetime

from ...models.textbook import Textbook
from ...models.chapter import Chapter
from ...models.section import Section
from ...models.interactive_element import InteractiveElement
from ...services.textbook_service import TextbookService
from ...utils.storage import FileStorage
from ...exceptions import TextbookNotFoundError, ValidationError
from ..base_service import BaseService


class ContentTypeSelectionService(BaseService):
    """
    Service for selecting and managing content types in textbooks.
    """

    def __init__(self):
        super().__init__()
        self.textbook_service = TextbookService()
        self.storage = FileStorage()

        # Define available content types and their characteristics
        self.content_types = {
            "text": {
                "name": "Text Content",
                "description": "Written content with explanations and examples",
                "default_inclusion": True,
                "customizable": False
            },
            "images": {
                "name": "Images and Diagrams",
                "description": "Visual content to support learning",
                "default_inclusion": False,
                "customizable": True
            },
            "quizzes": {
                "name": "Quizzes and Questions",
                "description": "Interactive assessments to test understanding",
                "default_inclusion": True,
                "customizable": True
            },
            "videos": {
                "name": "Video Content",
                "description": "Video explanations and demonstrations",
                "default_inclusion": False,
                "customizable": True
            },
            "exercises": {
                "name": "Practice Exercises",
                "description": "Hands-on activities to reinforce learning",
                "default_inclusion": True,
                "customizable": True
            },
            "summaries": {
                "name": "Chapter Summaries",
                "description": "Concise overviews of key concepts",
                "default_inclusion": True,
                "customizable": False
            },
            "glossary": {
                "name": "Glossary",
                "description": "Definitions of key terms",
                "default_inclusion": False,
                "customizable": False
            },
            "references": {
                "name": "References and Citations",
                "description": "Sources and additional reading materials",
                "default_inclusion": False,
                "customizable": False
            }
        }

    def select_content_types(
        self,
        textbook_id: str,
        selected_types: List[str],
        apply_immediately: bool = True
    ) -> Textbook:
        """
        Select content types for a textbook.

        Args:
            textbook_id: ID of the textbook to update
            selected_types: List of content types to include
            apply_immediately: Whether to apply changes immediately

        Returns:
            Updated textbook with selected content types
        """
        # Validate content types
        self._validate_content_types(selected_types)

        # Get the textbook
        textbook = self.textbook_service.get_textbook(textbook_id)

        # Update the textbook's metadata with selected content types
        if not textbook.metadata:
            textbook.metadata = {}
        textbook.metadata['selected_content_types'] = selected_types

        # Update the textbook in storage if applying immediately
        if apply_immediately:
            textbook.updated_at = datetime.now()
            textbook_data = textbook.model_dump()
            self.storage.save_textbook(textbook_id, textbook_data)

        return textbook

    def _validate_content_types(self, content_types: List[str]) -> None:
        """
        Validate the list of content types.

        Args:
            content_types: List of content types to validate

        Raises:
            ValidationError: If any content type is invalid
        """
        valid_types = set(self.content_types.keys())

        for content_type in content_types:
            if content_type not in valid_types:
                raise ValidationError(f"Invalid content type: {content_type}. Valid types are: {', '.join(valid_types)}")

    def get_available_content_types(self) -> Dict[str, Dict[str, Any]]:
        """
        Get all available content types.

        Returns:
            Dictionary of available content types with their details
        """
        return self.content_types.copy()

    def get_content_type_info(self, content_type: str) -> Dict[str, Any]:
        """
        Get information about a specific content type.

        Args:
            content_type: Name of the content type

        Returns:
            Information about the content type
        """
        if content_type not in self.content_types:
            raise ValidationError(f"Unknown content type: {content_type}")

        return self.content_types[content_type]

    def get_default_content_types_for_difficulty(self, difficulty: str) -> List[str]:
        """
        Get default content types based on difficulty level.

        Args:
            difficulty: Difficulty level (beginner, intermediate, advanced)

        Returns:
            List of default content types for the difficulty level
        """
        difficulty_defaults = {
            "beginner": ["text", "images", "quizzes", "exercises", "summaries"],
            "intermediate": ["text", "quizzes", "exercises", "summaries"],
            "advanced": ["text", "exercises", "references"]
        }

        return difficulty_defaults.get(difficulty, difficulty_defaults["intermediate"])

    def filter_content_by_type(self, textbook: Textbook, content_types: List[str]) -> Textbook:
        """
        Filter textbook content based on selected content types.

        Args:
            textbook: Original textbook
            content_types: List of content types to keep

        Returns:
            Filtered textbook containing only specified content types
        """
        # Create a new textbook instance with filtered content
        filtered_textbook = textbook.copy()

        # If 'text' is not in content_types, we still keep it as the base
        if 'text' not in content_types:
            content_types = content_types + ['text']

        # Filter chapters based on content types
        for chapter in filtered_textbook.chapters:
            # Filter sections based on content types
            # For now, keep all sections but this is where we'd apply filtering logic
            for section in chapter.sections:
                # Filter interactive elements based on content types
                if section.interactive_elements:
                    filtered_elements = []
                    for element in section.interactive_elements:
                        # Map content types to element types
                        element_type_mapping = {
                            'quizzes': ['quiz', 'question'],
                            'exercises': ['exercise', 'practice'],
                            'summaries': ['summary', 'overview']
                        }

                        include_element = False
                        for content_type in content_types:
                            if content_type in element_type_mapping:
                                if element.type in element_type_mapping[content_type]:
                                    include_element = True
                                    break
                            elif content_type == 'text' and element.type in ['text', 'content']:
                                include_element = True
                            elif content_type == 'images' and element.type in ['image', 'diagram', 'visual']:
                                include_element = True

                        if include_element:
                            filtered_elements.append(element)

                    section.interactive_elements = filtered_elements

        return filtered_textbook

    def enhance_textbook_with_content_types(
        self,
        textbook: Textbook,
        content_types: List[str]
    ) -> Textbook:
        """
        Enhance a textbook by adding specified content types where appropriate.

        Args:
            textbook: Original textbook to enhance
            content_types: List of content types to add

        Returns:
            Enhanced textbook with additional content types
        """
        enhanced_textbook = textbook.copy()

        for content_type in content_types:
            if content_type == "images":
                enhanced_textbook = self._add_images_to_textbook(enhanced_textbook)
            elif content_type == "quizzes":
                enhanced_textbook = self._add_quizzes_to_textbook(enhanced_textbook)
            elif content_type == "exercises":
                enhanced_textbook = self._add_exercises_to_textbook(enhanced_textbook)
            elif content_type == "glossary":
                enhanced_textbook = self._add_glossary_to_textbook(enhanced_textbook)
            elif content_type == "references":
                enhanced_textbook = self._add_references_to_textbook(enhanced_textbook)

        return enhanced_textbook

    def _add_images_to_textbook(self, textbook: Textbook) -> Textbook:
        """
        Add placeholder images to a textbook.

        Args:
            textbook: Textbook to add images to

        Returns:
            Textbook with image placeholders
        """
        # This would integrate with an image service in a real implementation
        # For now, just add metadata indicating images should be included
        if not textbook.metadata:
            textbook.metadata = {}
        textbook.metadata['images_included'] = True

        return textbook

    def _add_quizzes_to_textbook(self, textbook: Textbook) -> Textbook:
        """
        Add quizzes to a textbook.

        Args:
            textbook: Textbook to add quizzes to

        Returns:
            Textbook with quizzes added
        """
        # Add quiz elements to sections that don't have them
        for chapter in textbook.chapters:
            for section in chapter.sections:
                # Check if this section already has quizzes
                has_quiz = any(element.type == 'quiz' for element in section.interactive_elements)

                if not has_quiz:
                    # Add a quiz element
                    quiz_element = InteractiveElement(
                        id=f"quiz-{section.id}-{len(section.interactive_elements) + 1}",
                        type='quiz',
                        content='[Quiz content would be generated here based on section content]',
                        position=len(section.interactive_elements) + 1
                    )
                    section.interactive_elements.append(quiz_element)

        return textbook

    def _add_exercises_to_textbook(self, textbook: Textbook) -> Textbook:
        """
        Add exercises to a textbook.

        Args:
            textbook: Textbook to add exercises to

        Returns:
            Textbook with exercises added
        """
        # Add exercise elements to chapters that don't have them
        for chapter in textbook.chapters:
            # Check if this chapter already has exercises
            has_exercise = False
            for section in chapter.sections:
                if any(element.type == 'exercise' for element in section.interactive_elements):
                    has_exercise = True
                    break

            if not has_exercise and chapter.sections:
                # Add an exercise to the last section of the chapter
                exercise_element = InteractiveElement(
                    id=f"exercise-{chapter.id}-{len(chapter.sections)}",
                    type='exercise',
                    content='[Exercise content would be generated here based on chapter content]',
                    position=len(chapter.sections[0].interactive_elements) + 1 if chapter.sections else 1
                )
                if chapter.sections:
                    chapter.sections[-1].interactive_elements.append(exercise_element)

        return textbook

    def _add_glossary_to_textbook(self, textbook: Textbook) -> Textbook:
        """
        Add glossary to a textbook.

        Args:
            textbook: Textbook to add glossary to

        Returns:
            Textbook with glossary added
        """
        # Add glossary information to metadata
        if not textbook.metadata:
            textbook.metadata = {}
        textbook.metadata['glossary_included'] = True

        return textbook

    def _add_references_to_textbook(self, textbook: Textbook) -> Textbook:
        """
        Add references to a textbook.

        Args:
            textbook: Textbook to add references to

        Returns:
            Textbook with references added
        """
        # Add reference information to metadata
        if not textbook.metadata:
            textbook.metadata = {}
        textbook.metadata['references_included'] = True

        return textbook

    def get_content_type_recommendations(
        self,
        subject: str,
        difficulty: str,
        target_audience: str = ""
    ) -> Dict[str, Any]:
        """
        Get recommendations for content types based on subject and difficulty.

        Args:
            subject: Subject area
            difficulty: Difficulty level
            target_audience: Target audience

        Returns:
            Dictionary with content type recommendations
        """
        # Base recommendations by difficulty
        recommendations = {
            "beginner": {
                "high_priority": ["text", "images", "quizzes", "summaries"],
                "medium_priority": ["exercises"],
                "low_priority": ["glossary", "references"],
                "rationale": "Beginner learners need visual aids and frequent assessments"
            },
            "intermediate": {
                "high_priority": ["text", "quizzes", "exercises"],
                "medium_priority": ["summaries", "references"],
                "low_priority": ["images", "glossary"],
                "rationale": "Intermediate learners benefit from practice and deeper content"
            },
            "advanced": {
                "high_priority": ["text", "exercises", "references"],
                "medium_priority": ["quizzes"],
                "low_priority": ["images", "summaries"],
                "rationale": "Advanced learners need in-depth content and research materials"
            }
        }

        # Adjust recommendations based on subject
        subject_adjustments = {
            "mathematics": {"high_priority": ["text", "exercises", "quizzes"]},
            "art": {"high_priority": ["images", "text", "exercises"]},
            "science": {"high_priority": ["text", "images", "experiments/exercises", "references"]}
        }

        # Get base recommendations
        base_recs = recommendations.get(difficulty, recommendations["intermediate"])

        # Apply subject adjustments if available
        if subject.lower() in subject_adjustments:
            subject_rec = subject_adjustments[subject.lower()]
            for key, value in subject_rec.items():
                base_recs[key] = value

        return base_recs