from fastapi import APIRouter, HTTPException
from typing import Dict, Any

from ..schemas.translation_schemas import TranslationRequest, TranslationResponse
from ..models.textbook import Textbook
from ..models.chapter import Chapter
from ..models.section import Section
from ..services.translation.translation_service import TranslationService
from ..services.textbook_service import TextbookService
from ..exceptions import ValidationError, TextbookNotFoundError

router = APIRouter(prefix="/translation", tags=["translation"])

# Initialize services
translation_service = TranslationService()
textbook_service = TextbookService()


@router.post("/translate", response_model=TranslationResponse)
async def translate_content(request: TranslationRequest):
    """
    Translate textbook content to the specified language.
    """
    try:
        # Validate target language
        if not request.target_language or len(request.target_language) > 10:
            raise ValidationError("Invalid target language code")

        # Get textbook
        textbook = textbook_service.get_textbook(request.textbook_id)
        if not textbook:
            raise TextbookNotFoundError(f"Textbook with ID {request.textbook_id} not found")

        # Apply translation based on content type
        if request.content_type == "textbook":
            # For textbook translation, translate the entire textbook
            translated_textbook = await translation_service.translate_textbook(textbook, request.target_language)
            # Return a response for the main content being translated
            response = await translation_service.translate_content(request)
            return response
        else:
            # Translate the content directly based on the request
            response = await translation_service.translate_content(request)
            return response  # This is already a TranslationResponse object

    except ValidationError as e:
        raise HTTPException(status_code=422, detail=str(e))
    except TextbookNotFoundError:
        raise HTTPException(status_code=404, detail=f"Textbook with ID {request.textbook_id} not found")
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to translate content: {str(e)}")


@router.get("/status/{content_id}/{target_language}", response_model=Dict[str, Any])
async def get_translation_status(content_id: str, target_language: str):
    """
    Get the translation status for specific content.
    """
    try:
        # In a real implementation, we would check the actual translation status
        # For now, we'll return a placeholder response
        status = await translation_service.get_translation_status(content_id, target_language)

        return {
            "content_id": content_id,
            "target_language": target_language,
            "status": status.get("status", "not_cached"),
            "translated_content_available": status.get("translated_content_available", False),
            "expires_at": status.get("expires_at")
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to get translation status: {str(e)}")


@router.get("/languages", response_model=Dict[str, Any])
async def get_supported_languages():
    """
    Get list of supported translation languages.
    """
    try:
        # Return supported languages with special emphasis on Urdu for the hackathon
        supported_languages = {
            "languages": [
                {"code": "ur", "name": "Urdu"},
                {"code": "en", "name": "English"},
                {"code": "es", "name": "Spanish"},
                {"code": "fr", "name": "French"},
                {"code": "de", "name": "German"},
                {"code": "zh", "name": "Chinese"},
                {"code": "ja", "name": "Japanese"}
            ],
            "default_target_language": "ur",
            "message": "Urdu translation is specially optimized for Physical AI & Humanoid Robotics content"
        }

        return supported_languages
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to get supported languages: {str(e)}")