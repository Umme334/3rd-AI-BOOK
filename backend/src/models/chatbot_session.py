from pydantic import BaseModel
from typing import List, Optional, Dict, Any
from datetime import datetime


class ChatbotMessage(BaseModel):
    """
    Represents a single message in a chatbot conversation.
    """
    id: str
    role: str  # "user" or "assistant"
    content: str
    timestamp: datetime = None
    metadata: Optional[Dict[str, Any]] = {}
    query_analysis: Optional[Dict[str, Any]] = None  # For user messages
    context_sources: Optional[List[str]] = None  # For assistant messages

    def __init__(self, **data):
        super().__init__(**data)
        if self.timestamp is None:
            self.timestamp = datetime.now()


class ChatbotSession(BaseModel):
    """
    Represents a RAG chatbot session for answering questions about textbook content.
    """
    id: str
    user_id: Optional[str] = None
    textbook_id: str
    messages: List[ChatbotMessage] = []
    context_chunks: List[Dict[str, Any]] = []  # Retrieved context from RAG
    created_at: datetime = None
    updated_at: datetime = None
    is_active: bool = True

    def __init__(self, **data):
        super().__init__(**data)
        if self.created_at is None:
            self.created_at = datetime.now()
        self.updated_at = datetime.now()