import uuid
from typing import Dict, Any, Optional
from datetime import datetime

from ..models.textbook import Textbook, TextbookStatus
from ..utils.storage import FileStorage
from ..exceptions import TextbookNotFoundError, ValidationError
from .base_service import BaseService
from ..schemas.textbook_request import TextbookCreateRequest


class TextbookService(BaseService):
    """
    Service for managing textbook operations including creation, retrieval, and updates.
    """

    def __init__(self):
        super().__init__()
        self.storage = FileStorage()
        self._active_generations = {}  # Track active generation processes

    def create_textbook(self, request: TextbookCreateRequest) -> Textbook:
        """
        Create a new textbook based on the provided request.

        Args:
            request: Textbook creation request with parameters

        Returns:
            Created textbook object
        """
        # Validate the request
        self._validate_textbook_request(request)

        # Create a new textbook instance
        textbook_id = str(uuid.uuid4())
        textbook = Textbook(
            id=textbook_id,
            title=request.title,
            subject=request.subject,
            difficulty=request.difficulty,
            target_audience=request.target_audience,
            status=TextbookStatus.draft
        )

        # Save the textbook to storage
        textbook_data = textbook.model_dump()
        self.storage.save_textbook(textbook_id, textbook_data)

        return textbook

    def get_textbook(self, textbook_id: str) -> Textbook:
        """
        Retrieve a textbook by its ID.

        Args:
            textbook_id: Unique identifier of the textbook

        Returns:
            Textbook object

        Raises:
            TextbookNotFoundError: If the textbook doesn't exist
        """
        textbook_data = self.storage.load_textbook(textbook_id)
        if not textbook_data:
            raise TextbookNotFoundError(textbook_id)

        # Convert the loaded data back to a Textbook object
        return Textbook(**textbook_data)

    def update_textbook_status(self, textbook_id: str, status: TextbookStatus) -> Textbook:
        """
        Update the status of a textbook.

        Args:
            textbook_id: Unique identifier of the textbook
            status: New status to set

        Returns:
            Updated textbook object
        """
        textbook = self.get_textbook(textbook_id)
        textbook.status = status
        textbook.updated_at = datetime.now()

        # Save the updated textbook to storage
        textbook_data = textbook.model_dump()
        self.storage.save_textbook(textbook_id, textbook_data)

        return textbook

    def start_generation(self, textbook_id: str) -> bool:
        """
        Mark a textbook as being generated to prevent concurrent generation attempts.

        Args:
            textbook_id: Unique identifier of the textbook

        Returns:
            True if the generation was started, False if already in progress
        """
        if textbook_id in self._active_generations:
            return False

        self._active_generations[textbook_id] = datetime.now()
        self.update_textbook_status(textbook_id, TextbookStatus.generating)
        return True

    def finish_generation(self, textbook_id: str, success: bool = True) -> bool:
        """
        Mark a textbook generation as finished.

        Args:
            textbook_id: Unique identifier of the textbook
            success: Whether the generation was successful

        Returns:
            True if the generation was properly finished
        """
        if textbook_id not in self._active_generations:
            return False

        del self._active_generations[textbook_id]
        status = TextbookStatus.complete if success else TextbookStatus.failed
        self.update_textbook_status(textbook_id, status)
        return True

    def is_generation_active(self, textbook_id: str) -> bool:
        """
        Check if a textbook is currently being generated.

        Args:
            textbook_id: Unique identifier of the textbook

        Returns:
            True if generation is active, False otherwise
        """
        return textbook_id in self._active_generations

    def _validate_textbook_request(self, request: TextbookCreateRequest) -> None:
        """
        Validate the textbook creation request.

        Args:
            request: Textbook creation request to validate

        Raises:
            ValidationError: If the request is invalid
        """
        # Validate title
        if not request.title or len(request.title.strip()) == 0:
            raise ValidationError("Title is required and cannot be empty")

        if len(request.title) > 200:
            raise ValidationError("Title must be 200 characters or less")

        # Validate subject
        if not request.subject or len(request.subject.strip()) == 0:
            raise ValidationError("Subject is required and cannot be empty")

        # Validate difficulty
        if not request.difficulty:
            raise ValidationError("Difficulty level is required")

        # Validate chapter count if present
        if hasattr(request, 'chapter_count'):
            if request.chapter_count < 1 or request.chapter_count > 20:
                raise ValidationError("Chapter count must be between 1 and 20")

    def execute(self, *args, **kwargs):
        """
        Execute the service operation (required by BaseService).
        This is a placeholder implementation that can be overridden by subclasses.
        """
        # This is a general execute method that can be customized per service
        # For TextbookService, we might use this to execute a general operation
        # For now, we'll just return a default response
        raise NotImplementedError("Execute method should be implemented by specific operations")