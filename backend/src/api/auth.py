from fastapi import APIRouter, HTTPException, Depends, Request
from typing import Dict, Any
from datetime import datetime
import uuid
import httpx
from ..schemas.auth_schemas import (
    SignupRequest,
    SigninRequest,
    AuthResponse,
    SignupResponse,
    UserProfileUpdateRequest,
    UserProfileResponse
)
from ..models.user_profile import UserProfile
from ..services.auth.user_service import UserService
from ..services.auth.background_capture_service import BackgroundCaptureService
from ..exceptions import ValidationError, UserNotFoundError

router = APIRouter(prefix="/auth", tags=["auth"])

# Initialize services
user_service = UserService()
background_service = BackgroundCaptureService()

# Better Auth integration - validate session from better-auth.com
async def validate_better_auth_session(request: Request):
    """
    Validates the session token from better-auth.com
    """
    # Get session token from Authorization header
    auth_header = request.headers.get("Authorization")
    if not auth_header or not auth_header.startswith("Bearer "):
        raise HTTPException(status_code=401, detail="No valid session token provided")

    session_token = auth_header[7:]  # Remove "Bearer " prefix

    # In a real implementation, you would call better-auth.com's session validation API
    # For now, we'll simulate the validation by checking if the token exists in our system
    user_data = user_service.get_user_by_token(session_token)
    if not user_data:
        raise HTTPException(status_code=401, detail="Invalid or expired session")

    return user_data

@router.get("/better-auth/session")
async def get_better_auth_session(request: Request):
    """
    Endpoint to validate better-auth.com session and return user data.
    This endpoint is called by the frontend to validate the session.
    """
    try:
        user_data = await validate_better_auth_session(request)
        profile_response = await user_service.get_user_profile_response(user_data["id"])

        return {
            "user": {
                "id": user_data["id"],
                "email": user_data["email"],
                "name": f"{user_data.get('first_name', '')} {user_data.get('last_name', '')}".strip()
            },
            "user_profile": profile_response,
            "valid": True
        }
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to validate session: {str(e)}")

@router.post("/better-auth/callback")
async def better_auth_callback(request: Request):
    """
    Callback endpoint for better-auth.com to notify about authentication events.
    This could be used for account creation, updates, or other auth events.
    """
    try:
        # In a real implementation, better-auth.com would send data about the auth event
        # For now, we'll just return a success response
        body = await request.json()

        # Process the callback data (user created/updated/deleted)
        # This is where you'd sync user data between better-auth.com and your system
        event_type = body.get("event_type")
        user_info = body.get("user")

        if event_type == "user.created":
            # Handle user creation - maybe create a profile in your system
            pass
        elif event_type == "user.updated":
            # Handle user updates
            pass
        elif event_type == "user.deleted":
            # Handle user deletion
            pass

        return {"status": "success", "message": "Callback processed successfully"}
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to process callback: {str(e)}")

@router.post("/signup", response_model=SignupResponse)
async def signup(signup_request: SignupRequest):
    """
    Register a new user with background information.
    This endpoint is used when not using better-auth.com, or as a fallback.
    """
    try:
        # Validate background information
        background_data = {
            "software_experience": signup_request.software_experience,
            "hardware_experience": signup_request.hardware_experience,
            "programming_languages": signup_request.programming_languages,
            "hardware_platforms": signup_request.hardware_platforms,
            "robotics_experience": signup_request.robotics_experience,
            "math_background": signup_request.math_background,
            "primary_goal": signup_request.primary_goal,
            "background_questions": signup_request.background_questions
        }

        validation_errors = background_service.validate_background_info(background_data)
        has_errors = any(validation_errors.values())

        if has_errors:
            error_messages = []
            for category, errors in validation_errors.items():
                error_messages.extend(errors)
            if error_messages:
                raise ValidationError(", ".join(error_messages))

        # Create user
        user_data = user_service.create_user(signup_request)

        # Create access token
        token_data = user_service.create_access_token(user_data["user_id"])

        return SignupResponse(
            user_id=user_data["user_id"],
            email=user_data["email"],
            access_token=token_data["access_token"],
            token_type=token_data["token_type"],
            profile_created=True,
            message="User registered successfully"
        )
    except ValidationError as e:
        raise HTTPException(status_code=422, detail=str(e))
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to create user: {str(e)}")


@router.post("/signin", response_model=AuthResponse)
async def signin(signin_request: SigninRequest):
    """
    Authenticate an existing user.
    This endpoint is used when not using better-auth.com, or as a fallback.
    """
    try:
        # Authenticate user
        user_data = user_service.authenticate_user(signin_request)

        if not user_data:
            raise HTTPException(status_code=401, detail="Invalid email or password")

        # Create access token
        token_data = user_service.create_access_token(user_data["id"])

        # Get user profile
        profile_response = await user_service.get_user_profile_response(user_data["id"])

        return AuthResponse(
            user_id=user_data["id"],
            email=user_data["email"],
            access_token=token_data["access_token"],
            refresh_token=token_data.get("refresh_token"),
            token_type=token_data["token_type"],
            expires_at=token_data["expires_at"],
            user_profile=profile_response
        )
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to authenticate user: {str(e)}")


@router.get("/profile/{user_id}", response_model=UserProfileResponse, dependencies=[Depends(validate_better_auth_session)])
async def get_user_profile(user_id: str, request: Request):
    """
    Get user profile information.
    Requires valid better-auth.com session.
    """
    try:
        # Get the authenticated user from the session validation
        auth_user = await validate_better_auth_session(request)

        # Only allow users to access their own profile, or implement proper permissions
        if auth_user["id"] != user_id:
            raise HTTPException(status_code=403, detail="Not authorized to access this profile")

        profile_response = await user_service.get_user_profile_response(user_id)
        if not profile_response:
            raise UserNotFoundError(f"User profile with ID {user_id} not found")
        return profile_response
    except UserNotFoundError:
        raise HTTPException(status_code=404, detail=f"User profile with ID {user_id} not found")
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to get user profile: {str(e)}")


@router.put("/profile/{user_id}", response_model=UserProfileResponse, dependencies=[Depends(validate_better_auth_session)])
async def update_user_profile(user_id: str, profile_update: UserProfileUpdateRequest, request: Request):
    """
    Update user profile information.
    Requires valid better-auth.com session.
    """
    try:
        # Get the authenticated user from the session validation
        auth_user = await validate_better_auth_session(request)

        # Only allow users to update their own profile
        if auth_user["id"] != user_id:
            raise HTTPException(status_code=403, detail="Not authorized to update this profile")

        profile_response = await user_service.update_user_profile(user_id, profile_update)
        if not profile_response:
            raise UserNotFoundError(f"User profile with ID {user_id} not found")
        return profile_response
    except UserNotFoundError:
        raise HTTPException(status_code=404, detail=f"User profile with ID {user_id} not found")
    except ValidationError as e:
        raise HTTPException(status_code=422, detail=str(e))
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to update user profile: {str(e)}")


@router.post("/signout")
async def signout(request: Request):
    """
    Sign out the current user by invalidating their session.
    """
    try:
        # Validate the current session to get user info
        user_data = await validate_better_auth_session(request)

        # In a real implementation with Better-Auth, we would call their signout endpoint
        # For now, we'll simulate clearing the session by removing the token from our system
        # The actual session management would be handled by Better-Auth

        # Here we're just returning a success response
        # The frontend would handle clearing the local session/cookie
        return {
            "message": "Successfully signed out",
            "user_id": user_data["id"],
            "signed_out_at": datetime.now()
        }
    except HTTPException:
        # If the session is already invalid, that's fine - user is effectively signed out
        return {
            "message": "Session already invalid or expired",
            "signed_out_at": datetime.now()
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to sign out: {str(e)}")