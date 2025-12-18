from fastapi import APIRouter, HTTPException, BackgroundTasks
from typing import Dict, Any, Optional
from datetime import datetime
import uuid

from ..schemas.chatbot_schemas import (
    ChatbotQueryRequest,
    ChatbotResponse,
    ChatbotSessionCreateRequest,
    ChatbotSessionResponse,
    ChatHistoryResponse
)
from ..models.chatbot_session import ChatbotSession
from ..services.chatbot.rag_service import RAGService
from ..services.chatbot.content_chunker import ContentChunker
from ..exceptions import ValidationError, TextbookNotFoundError
from ..models.textbook import Textbook

router = APIRouter(prefix="/chatbot", tags=["chatbot"])

# Initialize services
rag_service = RAGService()
content_chunker = ContentChunker()


@router.post("/query", response_model=ChatbotResponse)
async def chatbot_query(request: ChatbotQueryRequest):
    """
    Process a query using the RAG chatbot system.
    """
    try:
        # Validate that the textbook exists
        if not request.textbook_id:
            raise ValidationError("textbook_id is required")

        # Import textbook service to validate textbook exists and check if it's indexed
        from ..services.textbook_service import TextbookService
        from ..exceptions import TextbookNotFoundError

        textbook_service = TextbookService()

        try:
            textbook = textbook_service.get_textbook(request.textbook_id)

            # Check if textbook is indexed for RAG, if not, log a warning
            if not textbook.rag_indexed:
                # Optionally trigger indexing if not done yet, but for now just log
                print(f"Warning: Textbook {request.textbook_id} is not indexed for RAG. Consider running /index-textbook/{request.textbook_id}")
        except TextbookNotFoundError:
            raise HTTPException(status_code=404, detail=f"Textbook with ID {request.textbook_id} not found")

        # Get or create a chatbot session
        session_id = request.session_id
        if not session_id:
            session = await rag_service.create_chatbot_session(
                textbook_id=request.textbook_id
            )
            session_id = session.id
        else:
            # In a real implementation, we would retrieve the existing session
            # For now, we'll create a temporary session
            session = await rag_service.create_chatbot_session(
                textbook_id=request.textbook_id
            )

        # Process the query using RAG
        response_content = await rag_service.process_query(session, request.query, request.selected_text)

        # Create response
        response = ChatbotResponse(
            response=response_content,
            session_id=session_id,
            sources=[],  # This would be populated with actual sources in a full implementation
            context_sources=["textbook_content"],  # In a real implementation, this would be actual sources
            followup_questions=[],  # In a real implementation, this would be generated
            query_time=datetime.now(),
            confidence=0.85,
            tokens_used=len(response_content.split())
        )

        return response
    except ValidationError as e:
        raise HTTPException(status_code=422, detail=str(e))
    except TextbookNotFoundError:
        raise HTTPException(status_code=404, detail=f"Textbook with ID {request.textbook_id} not found")
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to process chatbot query: {str(e)}")


@router.post("/sessions", response_model=ChatbotSessionResponse)
async def create_chatbot_session(session_request: ChatbotSessionCreateRequest):
    """
    Create a new chatbot session for a textbook.
    """
    try:
        # Generate a new session ID if not provided
        session_id = session_request.session_id or f"session_{uuid.uuid4()}"

        # In a real implementation, we would create a proper session in the database
        # For now, we'll create a mock response with the expected structure
        session_response = ChatbotSessionResponse(
            session_id=session_id,
            textbook_id=session_request.textbook_id,
            user_id=session_request.user_id,
            session_name=session_request.session_name or f"Session {datetime.now().strftime('%Y-%m-%d %H:%M')}",
            created_at=datetime.now(),
            last_accessed=datetime.now(),
            message_count=0,
            active=True
        )

        return session_response
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to create chatbot session: {str(e)}")


@router.get("/sessions/{session_id}", response_model=ChatbotSessionResponse)
async def get_chatbot_session(session_id: str):
    """
    Get details of a specific chatbot session.
    """
    try:
        # In a real implementation, we would retrieve the session from storage
        # For now, we'll return a placeholder response with proper structure
        session_response = ChatbotSessionResponse(
            session_id=session_id,
            textbook_id="placeholder_textbook_id",
            user_id="placeholder_user_id",
            session_name=f"Session {session_id[:8]}",
            created_at=datetime.now(),
            last_accessed=datetime.now(),
            message_count=0,
            active=True
        )

        return session_response
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to get chatbot session: {str(e)}")


@router.get("/sessions/{session_id}/history", response_model=ChatHistoryResponse)
async def get_chatbot_history(session_id: str):
    """
    Get the chat history for a specific session.
    """
    try:
        # In a real implementation, this would fetch from the database
        # For now, we'll return an empty history with proper structure
        history_response = ChatHistoryResponse(
            session_id=session_id,
            messages=[],
            total_messages=0
        )

        return history_response
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to get chatbot history: {str(e)}")


@router.post("/index-textbook/{textbook_id}")
async def index_textbook_for_rag(textbook_id: str, background_tasks: BackgroundTasks):
    """
    Index a textbook for RAG search.
    """
    try:
        # Import the textbook service to fetch the actual textbook
        from ..services.textbook_service import TextbookService
        from ..exceptions import TextbookNotFoundError

        textbook_service = TextbookService()

        try:
            # Fetch the actual textbook from storage
            textbook = textbook_service.get_textbook(textbook_id)
        except TextbookNotFoundError:
            raise HTTPException(status_code=404, detail=f"Textbook with ID {textbook_id} not found")

        # Index the textbook content in the background
        indexing_success = await rag_service.index_textbook_content(textbook)

        if indexing_success:
            # Update the textbook's RAG indexing status
            textbook.rag_indexed = True
            textbook.rag_index_id = f"index_{textbook_id}"
            textbook.rag_last_indexed = datetime.now()

            # Save the updated textbook back to storage
            textbook_service.storage.save_textbook(textbook_id, textbook.model_dump())

            return {
                "message": f"Successfully indexed textbook {textbook_id} for RAG",
                "textbook_id": textbook_id,
                "indexed_at": datetime.now(),
                "indexed_chapters_count": len(textbook.chapters)
            }
        else:
            raise HTTPException(status_code=500, detail="Failed to index textbook content")

    except HTTPException:
        raise  # Re-raise HTTP exceptions
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error indexing textbook: {str(e)}")


@router.delete("/sessions/{session_id}")
async def delete_chatbot_session(session_id: str):
    """
    Deactivate a chatbot session.
    """
    try:
        # In a real implementation, this would update the database
        # For now, we'll just return a success message
        return {
            "message": f"Session {session_id} deactivated",
            "session_id": session_id
        }

    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error deactivating session: {str(e)}")