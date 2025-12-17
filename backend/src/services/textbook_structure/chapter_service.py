from typing import Dict, Any, List
from datetime import datetime

from ...models.textbook import Textbook
from ...models.chapter import Chapter
from ...services.textbook_service import TextbookService
from ...utils.storage import FileStorage
from ...exceptions import TextbookNotFoundError, ValidationError
from ..base_service import BaseService


class ChapterOrganizationService(BaseService):
    """
    Service for customizing chapter organization and structure.
    """

    def __init__(self):
        super().__init__()
        self.textbook_service = TextbookService()
        self.storage = FileStorage()

    def reorganize_chapters(
        self,
        textbook_id: str,
        new_organization: Dict[str, Any],
        apply_immediately: bool = True
    ) -> Textbook:
        """
        Reorganize chapters in a textbook according to the new organization.

        Args:
            textbook_id: ID of the textbook to reorganize
            new_organization: Dictionary defining the new organization
            apply_immediately: Whether to apply changes immediately or store for later

        Returns:
            Updated textbook with reorganized chapters
        """
        # Validate organization structure
        self._validate_organization(new_organization)

        # Get the textbook
        textbook = self.textbook_service.get_textbook(textbook_id)

        # Apply the organization
        if 'order' in new_organization:
            textbook = self._apply_chapter_order(textbook, new_organization['order'])

        if 'titles' in new_organization:
            textbook = self._apply_chapter_titles(textbook, new_organization['titles'])

        if 'grouping' in new_organization:
            textbook = self._apply_chapter_grouping(textbook, new_organization['grouping'])

        # Update the textbook in storage if applying immediately
        if apply_immediately:
            textbook.updated_at = datetime.now()
            textbook_data = textbook.model_dump()
            self.storage.save_textbook(textbook_id, textbook_data)

        return textbook

    def _validate_organization(self, organization: Dict[str, Any]) -> None:
        """
        Validate the chapter organization structure.

        Args:
            organization: Organization structure to validate

        Raises:
            ValidationError: If the organization is invalid
        """
        # Validate order if provided
        if 'order' in organization:
            order = organization['order']
            if not isinstance(order, list):
                raise ValidationError("Chapter order must be a list of positions")

            # Check for duplicates
            if len(order) != len(set(order)):
                raise ValidationError("Chapter order cannot contain duplicate positions")

            # Check that all positions are positive integers
            for pos in order:
                if not isinstance(pos, int) or pos < 1:
                    raise ValidationError(f"All chapter positions must be positive integers, got: {pos}")

        # Validate titles if provided
        if 'titles' in organization:
            titles = organization['titles']
            if not isinstance(titles, list):
                raise ValidationError("Chapter titles must be a list")

            for title in titles:
                if not isinstance(title, str) or not title.strip():
                    raise ValidationError("Chapter titles must be non-empty strings")

        # Validate grouping if provided
        if 'grouping' in organization:
            grouping = organization['grouping']
            if not isinstance(grouping, dict):
                raise ValidationError("Chapter grouping must be a dictionary")

            if 'groups' not in grouping:
                raise ValidationError("Chapter grouping must contain 'groups' key")

    def _apply_chapter_order(self, textbook: Textbook, new_order: List[int]) -> Textbook:
        """
        Apply a new order to the chapters.

        Args:
            textbook: Original textbook
            new_order: List of chapter positions in new order

        Returns:
            Textbook with reordered chapters
        """
        if not new_order:
            return textbook

        # Validate that all positions are valid
        max_position = len(textbook.chapters)
        for pos in new_order:
            if pos < 1 or pos > max_position:
                raise ValidationError(f"Chapter position {pos} is out of range (1-{max_position})")

        # Create reordered chapters list
        reordered_chapters = []
        for pos in new_order:
            # Convert 1-based to 0-based index
            if 1 <= pos <= len(textbook.chapters):
                reordered_chapters.append(textbook.chapters[pos - 1])

        # Update positions
        for i, chapter in enumerate(reordered_chapters):
            chapter.position = i + 1

        textbook.chapters = reordered_chapters
        return textbook

    def _apply_chapter_titles(self, textbook: Textbook, new_titles: List[str]) -> Textbook:
        """
        Apply new titles to chapters.

        Args:
            textbook: Original textbook
            new_titles: List of new titles

        Returns:
            Textbook with updated chapter titles
        """
        min_len = min(len(textbook.chapters), len(new_titles))
        for i in range(min_len):
            textbook.chapters[i].title = new_titles[i]

        return textbook

    def _apply_chapter_grouping(self, textbook: Textbook, grouping: Dict[str, Any]) -> Textbook:
        """
        Apply grouping structure to chapters.

        Args:
            textbook: Original textbook
            grouping: Grouping structure

        Returns:
            Textbook with chapters organized into groups
        """
        # Store grouping information in textbook metadata
        if not textbook.metadata:
            textbook.metadata = {}

        textbook.metadata['chapter_grouping'] = grouping
        return textbook

    def suggest_chapter_organization(
        self,
        subject: str,
        difficulty: str,
        chapter_count: int = 5
    ) -> Dict[str, Any]:
        """
        Suggest an appropriate chapter organization for a subject and difficulty level.

        Args:
            subject: Subject area
            difficulty: Difficulty level
            chapter_count: Number of chapters

        Returns:
            Dictionary with suggested organization
        """
        # Base organization patterns
        base_patterns = {
            "beginner": {
                "order": list(range(1, chapter_count + 1)),
                "structure": "progressive_building",
                "titles": [f"Introduction to {subject}", f"Basic Concepts of {subject}", f"Fundamentals of {subject}", f"Applications of {subject}", f"Review and Practice"],
                "grouping": {"groups": [{"name": "Foundations", "chapters": [1, 2]}, {"name": "Application", "chapters": [3, 4]}, {"name": "Synthesis", "chapters": [5]}]}
            },
            "intermediate": {
                "order": list(range(1, chapter_count + 1)),
                "structure": "thematic",
                "titles": [f"Theory of {subject}", f"Methods in {subject}", f"Case Studies in {subject}", f"Advanced Concepts", f"Integration and Practice"],
                "grouping": {"groups": [{"name": "Theory", "chapters": [1]}, {"name": "Practice", "chapters": [2, 3]}, {"name": "Advanced", "chapters": [4, 5]}]}
            },
            "advanced": {
                "order": list(range(1, chapter_count + 1)),
                "structure": "research_oriented",
                "titles": [f"Literature Review: {subject}", f"Methodology in {subject}", f"Current Research", f"Critical Analysis", f"Future Directions"],
                "grouping": {"groups": [{"name": "Foundation", "chapters": [1]}, {"name": "Research", "chapters": [2, 3]}, {"name": "Analysis", "chapters": [4]}, {"name": "Outlook", "chapters": [5]}]}
            }
        }

        # Get the pattern for the difficulty level
        pattern = base_patterns.get(difficulty, base_patterns["intermediate"])

        # Adjust for the actual chapter count if different from default
        if chapter_count != 5:
            # Create a simple sequential order
            pattern["order"] = list(range(1, chapter_count + 1))

            # Adjust titles if needed
            pattern["titles"] = [f"Chapter {i+1}: {pattern['titles'][min(i, len(pattern['titles'])-1)]}" for i in range(chapter_count)]

            # Adjust grouping
            if chapter_count <= 3:
                pattern["grouping"] = {"groups": [{"name": "Main Content", "chapters": list(range(1, chapter_count + 1))}]}
            else:
                # Divide into roughly equal groups
                group_size = max(2, chapter_count // 3)
                groups = []
                for i in range(0, chapter_count, group_size):
                    end = min(i + group_size, chapter_count)
                    groups.append({
                        "name": f"Section {len(groups) + 1}",
                        "chapters": list(range(i + 1, end + 1))
                    })
                pattern["grouping"]["groups"] = groups

        return pattern

    def validate_chapter_organization(
        self,
        textbook: Textbook,
        organization: Dict[str, Any]
    ) -> List[str]:
        """
        Validate if a chapter organization is compatible with a textbook.

        Args:
            textbook: Textbook to validate organization for
            organization: Organization to validate

        Returns:
            List of validation issues (empty if valid)
        """
        issues = []

        # Check if the organization references valid chapter positions
        if 'order' in organization:
            max_chapter_pos = len(textbook.chapters)
            for pos in organization['order']:
                if pos < 1 or pos > max_chapter_pos:
                    issues.append(f"Chapter position {pos} is out of range for textbook with {max_chapter_pos} chapters")

        # Check if titles count matches chapter count
        if 'titles' in organization and len(organization['titles']) != len(textbook.chapters):
            issues.append(f"Number of titles ({len(organization['titles'])}) doesn't match number of chapters ({len(textbook.chapters)})")

        return issues