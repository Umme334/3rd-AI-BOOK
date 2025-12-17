from pydantic import BaseModel
from typing import Optional, List, Dict, Any
from datetime import datetime


class ChatbotQueryRequest(BaseModel):
    """
    Request schema for chatbot queries
    """
    query: str
    textbook_id: str
    session_id: Optional[str] = None
    user_id: Optional[str] = None
    context_window: Optional[int] = 5  # Number of previous messages to include
    temperature: Optional[float] = 0.7  # Controls randomness in responses


class ChatbotResponse(BaseModel):
    """
    Response schema for chatbot queries
    """
    response: str
    session_id: str
    query_time: datetime
    sources: List[Dict[str, Any]]  # List of source references used to generate response
    confidence: float  # Confidence score for the response (0-1)
    tokens_used: int  # Number of tokens used in the response


class ChatbotSessionCreateRequest(BaseModel):
    """
    Request schema for creating a new chatbot session
    """
    textbook_id: str
    user_id: Optional[str] = None
    session_name: Optional[str] = None
    initial_context: Optional[Dict[str, Any]] = None


class ChatbotSessionResponse(BaseModel):
    """
    Response schema for chatbot session information
    """
    session_id: str
    textbook_id: str
    user_id: Optional[str]
    session_name: Optional[str]
    created_at: datetime
    last_accessed: datetime
    message_count: int
    active: bool


class ChatHistoryResponse(BaseModel):
    """
    Response schema for chat history
    """
    session_id: str
    messages: List[Dict[str, Any]]  # List of {role, content, timestamp}
    total_messages: int