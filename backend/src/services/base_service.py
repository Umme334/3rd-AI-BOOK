from abc import ABC, abstractmethod
from typing import Any, Dict, Optional
from datetime import datetime


class BaseService(ABC):
    """
    Base service class providing common functionality for all services.
    """

    def __init__(self):
        self.created_at = datetime.now()

    def _validate_required_fields(self, data: Dict[str, Any], required_fields: list) -> None:
        """
        Validate that required fields are present in the data.

        Args:
            data: Dictionary containing the data to validate
            required_fields: List of required field names

        Raises:
            ValueError: If any required field is missing
        """
        missing_fields = [field for field in required_fields if field not in data or data[field] is None]
        if missing_fields:
            raise ValueError(f"Missing required fields: {', '.join(missing_fields)}")

    def _validate_field_type(self, value: Any, expected_type: type, field_name: str) -> None:
        """
        Validate that a field is of the expected type.

        Args:
            value: Value to validate
            expected_type: Expected type
            field_name: Name of the field being validated

        Raises:
            TypeError: If the value is not of the expected type
        """
        if not isinstance(value, expected_type):
            raise TypeError(f"Field '{field_name}' must be of type {expected_type.__name__}, got {type(value).__name__}")

    def _validate_string_length(self, value: str, min_length: int, max_length: int, field_name: str) -> None:
        """
        Validate the length of a string field.

        Args:
            value: String value to validate
            min_length: Minimum allowed length
            max_length: Maximum allowed length
            field_name: Name of the field being validated

        Raises:
            ValueError: If the string length is outside the allowed range
        """
        if len(value) < min_length:
            raise ValueError(f"Field '{field_name}' must be at least {min_length} characters long")
        if len(value) > max_length:
            raise ValueError(f"Field '{field_name}' must be no more than {max_length} characters long")

    def _sanitize_input(self, text: str) -> str:
        """
        Sanitize input text by removing potentially harmful content.

        Args:
            text: Text to sanitize

        Returns:
            Sanitized text
        """
        # Remove potential script tags and other harmful content
        sanitized = text.replace('<script', '&lt;script').replace('</script>', '&lt;/script&gt;')
        return sanitized.strip()

    @abstractmethod
    def execute(self, *args, **kwargs) -> Any:
        """
        Execute the service operation. This method must be implemented by subclasses.
        """
        pass