from typing import Dict, Any, List
from enum import Enum

from ..exceptions import ValidationError
from ..schemas.textbook_request import DifficultyLevel


class ValidationService:
    """
    Service for validating input parameters and content.
    """

    def __init__(self):
        self.valid_difficulty_levels = [level.value for level in DifficultyLevel]
        self.max_title_length = 200
        self.max_subject_length = 100
        self.max_target_audience_length = 500
        self.min_chapter_count = 1
        self.max_chapter_count = 20

    def validate_textbook_creation_request(self, data: Dict[str, Any]) -> None:
        """
        Validate textbook creation request parameters.

        Args:
            data: Dictionary containing request parameters

        Raises:
            ValidationError: If validation fails
        """
        # Validate title
        title = data.get('title', '')
        if not title or not title.strip():
            raise ValidationError("Title is required and cannot be empty")

        if len(title) > self.max_title_length:
            raise ValidationError(f"Title must be {self.max_title_length} characters or less")

        # Validate subject
        subject = data.get('subject', '')
        if not subject or not subject.strip():
            raise ValidationError("Subject is required and cannot be empty")

        if len(subject) > self.max_subject_length:
            raise ValidationError(f"Subject must be {self.max_subject_length} characters or less")

        # Validate difficulty
        difficulty = data.get('difficulty', '')
        if not difficulty:
            raise ValidationError("Difficulty level is required")

        if difficulty not in self.valid_difficulty_levels:
            raise ValidationError(f"Difficulty must be one of: {', '.join(self.valid_difficulty_levels)}")

        # Validate target audience if provided
        target_audience = data.get('target_audience', '')
        if target_audience and len(target_audience) > self.max_target_audience_length:
            raise ValidationError(f"Target audience must be {self.max_target_audience_length} characters or less")

        # Validate chapter count if provided
        chapter_count = data.get('chapter_count', 5)  # Default is 5
        if not isinstance(chapter_count, int):
            raise ValidationError("Chapter count must be an integer")

        if chapter_count < self.min_chapter_count or chapter_count > self.max_chapter_count:
            raise ValidationError(f"Chapter count must be between {self.min_chapter_count} and {self.max_chapter_count}")

        # Validate boolean flags
        include_quizzes = data.get('include_quizzes', True)
        include_summaries = data.get('include_summaries', True)

        if not isinstance(include_quizzes, bool):
            raise ValidationError("include_quizzes must be a boolean value")

        if not isinstance(include_summaries, bool):
            raise ValidationError("include_summaries must be a boolean value")

    def validate_textbook_structure_request(self, data: Dict[str, Any]) -> None:
        """
        Validate textbook structure customization request parameters.

        Args:
            data: Dictionary containing request parameters

        Raises:
            ValidationError: If validation fails
        """
        # Validate textbook ID
        textbook_id = data.get('textbook_id', '')
        if not textbook_id or not textbook_id.strip():
            raise ValidationError("Textbook ID is required and cannot be empty")

        # Validate chapter organization if provided
        chapter_organization = data.get('chapter_organization')
        if chapter_organization is not None and not isinstance(chapter_organization, dict):
            raise ValidationError("Chapter organization must be a dictionary")

        # Validate content types if provided
        content_types = data.get('content_types', [])
        if not isinstance(content_types, list):
            raise ValidationError("Content types must be a list")

        valid_content_types = ['text', 'images', 'quizzes', 'videos', 'exercises', 'summaries']
        for content_type in content_types:
            if not isinstance(content_type, str) or content_type not in valid_content_types:
                raise ValidationError(f"Content type '{content_type}' is not valid. Valid types are: {', '.join(valid_content_types)}")

        # Validate custom learning objectives if provided
        custom_learning_objectives = data.get('custom_learning_objectives')
        if custom_learning_objectives is not None:
            if not isinstance(custom_learning_objectives, list):
                raise ValidationError("Custom learning objectives must be a list")

            for obj in custom_learning_objectives:
                if not isinstance(obj, str) or not obj.strip():
                    raise ValidationError("Each learning objective must be a non-empty string")

        # Validate excluded chapters if provided
        exclude_chapters = data.get('exclude_chapters', [])
        if not isinstance(exclude_chapters, list):
            raise ValidationError("Excluded chapters must be a list")

        for chapter_pos in exclude_chapters:
            if not isinstance(chapter_pos, int) or chapter_pos < 1:
                raise ValidationError("Each excluded chapter position must be a positive integer")

    def validate_export_request(self, data: Dict[str, Any]) -> None:
        """
        Validate export request parameters.

        Args:
            data: Dictionary containing request parameters

        Raises:
            ValidationError: If validation fails
        """
        # Validate format
        export_format = data.get('format', '')
        if not export_format or not export_format.strip():
            raise ValidationError("Export format is required")

        valid_formats = ['pdf', 'html', 'markdown']
        if export_format.lower() not in valid_formats:
            raise ValidationError(f"Export format must be one of: {', '.join(valid_formats)}")

        # Validate include navigation if provided
        include_navigation = data.get('include_navigation', True)
        if not isinstance(include_navigation, bool):
            raise ValidationError("Include navigation must be a boolean value")

    def validate_content_quality(self, content: str, subject: str, difficulty: str) -> List[str]:
        """
        Validate the quality of generated content.

        Args:
            content: Content to validate
            subject: Subject area
            difficulty: Difficulty level

        Returns:
            List of validation issues found (empty list if content is valid)
        """
        issues = []

        if not content or len(content.strip()) == 0:
            issues.append("Content cannot be empty")
            return issues

        # Check for minimum length
        if len(content.split()) < 50:  # At least 50 words
            issues.append("Content appears to be too short for educational material")

        # Check for basic structure (contains sections)
        content_lower = content.lower()
        common_section_indicators = ['chapter', 'section', 'introduction', 'conclusion', 'summary']
        has_structure = any(indicator in content_lower for indicator in common_section_indicators)

        if not has_structure:
            issues.append("Content may be missing proper educational structure")

        # For now, return issues found
        # In a real implementation, this would include more sophisticated content validation

        return issues

    def sanitize_text_input(self, text: str) -> str:
        """
        Sanitize text input to prevent security issues.

        Args:
            text: Text to sanitize

        Returns:
            Sanitized text
        """
        if not text:
            return text

        # Remove potential script tags
        sanitized = text.replace('<script', '&lt;script').replace('</script>', '&lt;/script&gt;')

        # Remove other potentially harmful tags (basic implementation)
        harmful_tags = ['<iframe', '<object', '<embed', '<form']
        for tag in harmful_tags:
            sanitized = sanitized.replace(tag, f'&lt;{tag[1:]}')

        return sanitized.strip()