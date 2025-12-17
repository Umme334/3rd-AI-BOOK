from pydantic import BaseModel
from typing import Optional
from .textbook_response import TextbookStatus


class ProgressResponse(BaseModel):
    """
    Response schema for textbook generation progress.
    """
    id: str
    status: TextbookStatus
    progress: float  # Progress as a percentage (0.0 to 1.0)
    message: Optional[str] = None