from typing import Optional
from fastapi import HTTPException, status


class TextbookGenerationError(HTTPException):
    """Base exception for textbook generation errors."""

    def __init__(self, detail: str, status_code: int = status.HTTP_400_BAD_REQUEST):
        super().__init__(status_code=status_code, detail=detail)


class TextbookNotFoundError(TextbookGenerationError):
    """Raised when a textbook is not found."""

    def __init__(self, textbook_id: str):
        super().__init__(
            detail=f"Textbook with ID {textbook_id} not found",
            status_code=status.HTTP_404_NOT_FOUND
        )


class TextbookGenerationInProgressError(TextbookGenerationError):
    """Raised when trying to regenerate a textbook that is already being generated."""

    def __init__(self, textbook_id: str):
        super().__init__(
            detail=f"Textbook with ID {textbook_id} is already being generated",
            status_code=status.HTTP_409_CONFLICT
        )


class InvalidInputParametersError(TextbookGenerationError):
    """Raised when input parameters for textbook generation are invalid."""

    def __init__(self, detail: str):
        super().__init__(
            detail=f"Invalid input parameters: {detail}",
            status_code=status.HTTP_422_UNPROCESSABLE_ENTITY
        )


class ExportFormatNotSupportedError(TextbookGenerationError):
    """Raised when an unsupported export format is requested."""

    def __init__(self, format_type: str):
        super().__init__(
            detail=f"Export format '{format_type}' is not supported. Supported formats: pdf, html, markdown",
            status_code=status.HTTP_400_BAD_REQUEST
        )


class ContentGenerationError(TextbookGenerationError):
    """Raised when content generation fails."""

    def __init__(self, detail: str = "Content generation failed"):
        super().__init__(
            detail=detail,
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR
        )


class ValidationError(TextbookGenerationError):
    """Raised when validation fails."""

    def __init__(self, detail: str):
        super().__init__(
            detail=f"Validation error: {detail}",
            status_code=status.HTTP_422_UNPROCESSABLE_ENTITY
        )


class UserNotFoundError(TextbookGenerationError):
    """Raised when a user is not found."""

    def __init__(self, user_id: str):
        super().__init__(
            detail=f"User with ID {user_id} not found",
            status_code=status.HTTP_404_NOT_FOUND
        )


class ConfigurationError(Exception):
    """Raised when there's a configuration issue (e.g., missing API keys)."""

    def __init__(self, detail: str):
        self.detail = detail
        super().__init__(detail)