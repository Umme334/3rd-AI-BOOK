from fastapi import APIRouter, HTTPException
from typing import Dict, Any
from datetime import datetime

from ..schemas.personalization_schemas import (
    PersonalizationRequest,
    PersonalizationResponse,
    PersonalizationPreferencesRequest,
    PersonalizationPreferencesResponse
)
from ..models.textbook import Textbook
from ..models.user_profile import UserProfile
from ..services.personalization.personalization_service import PersonalizationService
from ..services.personalization.content_adapter import ContentAdapter
from ..services.auth.user_service import UserService
from ..services.textbook_service import TextbookService
from ..exceptions import ValidationError, UserNotFoundError, TextbookNotFoundError

router = APIRouter(prefix="/personalization", tags=["personalization"])

# Initialize services
personalization_service = PersonalizationService()
content_adapter = ContentAdapter()
user_service = UserService()
textbook_service = TextbookService()


@router.post("/apply", response_model=PersonalizationResponse)
async def apply_personalization(request: PersonalizationRequest):
    """
    Apply personalization to textbook content based on user profile.
    """
    try:
        # Use the new personalization service method
        response = await personalization_service.get_personalized_content(request)
        return response

    except ValidationError as e:
        raise HTTPException(status_code=422, detail=str(e))
    except UserNotFoundError:
        raise HTTPException(status_code=404, detail=f"User with ID {request.user_id} not found")
    except TextbookNotFoundError:
        raise HTTPException(status_code=404, detail=f"Textbook with ID {request.textbook_id} not found")
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to apply personalization: {str(e)}")


@router.post("/toggle")
async def toggle_personalization(user_id: str, textbook_id: str, enabled: bool):
    """
    Toggle personalization on/off for a user and textbook.
    """
    try:
        # In a real implementation, this would update the user's preferences in the database
        # For now, we'll just return a success message
        return {
            "user_id": user_id,
            "textbook_id": textbook_id,
            "personalization_enabled": enabled,
            "message": f"Personalization {'enabled' if enabled else 'disabled'} for user {user_id} and textbook {textbook_id}"
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to toggle personalization: {str(e)}")


@router.get("/user/{user_id}/preferences", response_model=PersonalizationPreferencesResponse)
async def get_user_personalization_preferences(user_id: str):
    """
    Get user's personalization preferences.
    """
    try:
        user_profile_response = await user_service.get_user_profile_response(user_id)
        if not user_profile_response:
            raise UserNotFoundError(f"User profile with ID {user_id} not found")

        # Map to PersonalizationPreferencesResponse
        preferences = PersonalizationPreferencesResponse(
            user_id=user_id,
            learning_style=None,  # Would come from specific learning style preferences
            content_pacing="standard",
            example_preference=None,  # Would be determined from user profile
            difficulty_tolerance=0.7,
            feedback_frequency="regular",
            accessibility_options={},
            updated_at=datetime.now()
        )

        return preferences
    except UserNotFoundError:
        raise HTTPException(status_code=404, detail=f"User with ID {user_id} not found")
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to get user preferences: {str(e)}")


@router.put("/user/{user_id}/preferences", response_model=PersonalizationPreferencesResponse)
async def update_user_personalization_preferences(user_id: str, preferences_request: PersonalizationPreferencesRequest):
    """
    Update user's personalization preferences.
    """
    try:
        # In a real implementation, this would update the user's preferences in the database
        # For now, we'll just return the updated preferences
        updated_preferences = PersonalizationPreferencesResponse(
            user_id=user_id,
            learning_style=preferences_request.learning_style,
            content_pacing=preferences_request.content_pacing or "standard",
            example_preference=preferences_request.example_preference,
            difficulty_tolerance=preferences_request.difficulty_tolerance or 0.7,
            feedback_frequency=preferences_request.feedback_frequency or "regular",
            accessibility_options=preferences_request.accessibility_options or {},
            updated_at=datetime.now()
        )

        return updated_preferences
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to update user preferences: {str(e)}")


@router.post("/recommend", response_model=Dict[str, Any])
async def get_personalization_recommendations(request: PersonalizationRequest):
    """
    Get personalization recommendations for content.
    """
    try:
        user_profile = await user_service.get_user_profile(request.user_id)
        if not user_profile:
            raise UserNotFoundError(f"User profile with ID {request.user_id} not found")

        textbook = textbook_service.get_textbook(request.textbook_id)
        if not textbook:
            raise TextbookNotFoundError(f"Textbook with ID {request.textbook_id} not found")

        # Generate recommendations based on user profile
        recommendations = {
            "user_id": user_profile.id,
            "textbook_id": textbook.id,
            "recommended_difficulty": personalization_service._calculate_difficulty_adjustment(user_profile),
            "recommended_modules": [],  # Would be populated based on user's background
            "suggested_prerequisites": [],  # Would be populated based on gaps in user's knowledge
            "personalization_score": 0.8  # Placeholder score
        }

        return recommendations
    except UserNotFoundError:
        raise HTTPException(status_code=404, detail=f"User with ID {request.user_id} not found")
    except TextbookNotFoundError:
        raise HTTPException(status_code=404, detail=f"Textbook with ID {request.textbook_id} not found")
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to get recommendations: {str(e)}")