from pydantic import BaseModel
from typing import Optional, List
from datetime import datetime


class ExportRequest(BaseModel):
    """
    Request schema for exporting a textbook.
    """
    format: str  # pdf, html, markdown
    include_navigation: bool = True
    include_interactive_elements: bool = True
    custom_stylesheet: Optional[str] = None  # Path to custom CSS for HTML/PDF


class ExportResponse(BaseModel):
    """
    Response schema for textbook export.
    """
    download_url: str
    format: str  # pdf, html, markdown
    file_size: int  # Size in bytes
    expires_at: datetime
    message: str = "Export completed successfully"