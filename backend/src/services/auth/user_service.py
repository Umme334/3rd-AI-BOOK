from typing import Optional, List, Dict, Any
from datetime import datetime, timedelta
import uuid
import secrets
from passlib.context import CryptContext

from ...models.user_profile import UserProfile
from ...utils.storage import FileStorage
from ...exceptions import UserNotFoundError, ValidationError
from ..base_service import BaseService
from ...schemas.auth_schemas import (
    SignupRequest,
    SigninRequest,
    UserProfileUpdateRequest,
    UserProfileResponse,
    AuthResponse
)


# Password hashing context
pwd_context = CryptContext(schemes=["bcrypt"], deprecated="auto")


class UserService(BaseService):
    """
    Service for user management and authentication.
    """

    def __init__(self):
        super().__init__()
        self.storage = FileStorage()
        # In a real implementation, this would connect to a database
        # For this implementation, we'll use an in-memory storage
        self.users_db = {}  # email -> user data
        self.sessions_db = {}  # session_token -> user_id
        self.password_reset_tokens = {}  # token -> user_id

    def create_user(self, signup_data: SignupRequest) -> Dict[str, Any]:
        """
        Create a new user account
        """
        # Check if user already exists
        if signup_data.email in self.users_db:
            raise ValueError("User with this email already exists")

        # Generate user ID
        user_id = str(uuid.uuid4())

        # Hash the password
        hashed_password = pwd_context.hash(signup_data.password)

        # Create user data
        user_data = {
            "id": user_id,
            "email": signup_data.email,
            "hashed_password": hashed_password,
            "first_name": signup_data.first_name,
            "last_name": signup_data.last_name,
            "created_at": datetime.now(),
            "updated_at": datetime.now(),
            "is_active": True,
            "email_verified": False,
            "last_login": None
        }

        # Store user
        self.users_db[signup_data.email] = user_data

        # Create user profile with background information
        profile_data = {
            "id": user_id,
            "email": signup_data.email,
            "name": f"{signup_data.first_name} {signup_data.last_name or ''}".strip(),
            "software_background": signup_data.software_experience or "beginner",
            "hardware_background": signup_data.hardware_experience or "none",
            "programming_languages": signup_data.programming_languages or [],
            "hardware_experience": signup_data.hardware_platforms or [],
            "robotics_experience": signup_data.robotics_experience or "none",
            "education_level": "undergraduate",  # Default, can be updated later
            "math_background": signup_data.math_background or "basic",
            "primary_goal": signup_data.primary_goal,
            "background_questions": signup_data.background_questions or {},
            "preferences": {},
            "created_at": datetime.now(),
            "updated_at": datetime.now()
        }

        # Save profile to storage
        self.storage.save_user_profile(user_id, profile_data)

        return {
            "user_id": user_id,
            "email": signup_data.email,
            "first_name": signup_data.first_name,
            "last_name": signup_data.last_name,
            "created_at": user_data["created_at"]
        }

    def authenticate_user(self, signin_data: SigninRequest) -> Optional[Dict[str, Any]]:
        """
        Authenticate a user with email and password
        """
        # Get user by email
        user_data = self.users_db.get(signin_data.email)
        if not user_data:
            return None

        # Verify password
        if not pwd_context.verify(signin_data.password, user_data["hashed_password"]):
            return None

        # Update last login
        user_data["updated_at"] = datetime.now()
        user_data["last_login"] = datetime.now()

        return user_data

    def create_access_token(self, user_id: str) -> Dict[str, Any]:
        """
        Create an access token for the user
        """
        # Generate token
        token = secrets.token_urlsafe(32)
        expires_at = datetime.now() + timedelta(hours=24)  # Token expires in 24 hours

        # Store session
        self.sessions_db[token] = {
            "user_id": user_id,
            "expires_at": expires_at,
            "created_at": datetime.now()
        }

        return {
            "access_token": token,
            "token_type": "bearer",
            "expires_at": expires_at
        }

    async def get_user_profile(self, user_id: str) -> UserProfile:
        """
        Get a user profile by ID.
        """
        try:
            profile_data = self.storage.get_user_profile(user_id)
            if not profile_data:
                raise UserNotFoundError(f"User profile not found: {user_id}")

            return UserProfile(**profile_data)
        except Exception as e:
            raise UserNotFoundError(f"User profile not found: {user_id}") from e

    def get_user_by_token(self, token: str) -> Optional[Dict[str, Any]]:
        """
        Get user information by access token
        """
        session_data = self.sessions_db.get(token)
        if not session_data:
            return None

        # Check if token is expired
        if session_data["expires_at"] < datetime.now():
            del self.sessions_db[token]
            return None

        # Get user data
        for user_data in self.users_db.values():
            if user_data["id"] == session_data["user_id"]:
                return user_data

        return None

    async def get_user_profile_response(self, user_id: str) -> Optional[UserProfileResponse]:
        """
        Get user profile as response object
        """
        try:
            profile = await self.get_user_profile(user_id)
            if not profile:
                return None

            # Map to UserProfileResponse
            return UserProfileResponse(
                user_id=profile.id,
                email=profile.email,
                first_name=profile.name.split()[0] if profile.name else "",
                last_name=" ".join(profile.name.split()[1:]) if profile.name and len(profile.name.split()) > 1 else None,
                created_at=profile.created_at,
                updated_at=profile.updated_at,
                software_experience=profile.software_background,
                hardware_experience=profile.hardware_background,
                programming_languages=profile.programming_languages,
                hardware_platforms=profile.hardware_experience,
                robotics_experience=profile.robotics_experience,
                math_background=getattr(profile, 'math_background', 'basic'),
                primary_goal=getattr(profile, 'primary_goal', None),
                background_questions=getattr(profile, 'background_questions', {}),
                preferences=getattr(profile, 'preferences', {}),
                profile_completed=bool(profile.software_background),
                last_login=self.users_db.get(profile.email, {}).get('last_login') if profile.email in self.users_db else None
            )
        except Exception as e:
            self.logger.error(f"Failed to get user profile response: {str(e)}")
            return None

    async def update_user_profile(self, user_id: str, profile_update: UserProfileUpdateRequest) -> Optional[UserProfileResponse]:
        """
        Update user profile information
        """
        profile = await self.get_user_profile(user_id)
        if not profile:
            return None

        # Update profile fields from the request
        if profile_update.first_name is not None or profile_update.last_name is not None:
            first_name = profile_update.first_name or profile.name.split()[0] if profile.name else ""
            last_name = profile_update.last_name or (" ".join(profile.name.split()[1:]) if profile.name and len(profile.name.split()) > 1 else "")
            profile.name = f"{first_name} {last_name}".strip()

        if profile_update.software_experience is not None:
            profile.software_background = profile_update.software_experience
        if profile_update.hardware_experience is not None:
            profile.hardware_background = profile_update.hardware_experience
        if profile_update.programming_languages is not None:
            profile.programming_languages = profile_update.programming_languages
        if profile_update.hardware_platforms is not None:
            profile.hardware_experience = profile_update.hardware_platforms
        if profile_update.robotics_experience is not None:
            profile.robotics_experience = profile_update.robotics_experience
        if profile_update.math_background is not None:
            # Add math_background to profile if it doesn't exist
            profile.__dict__['math_background'] = profile_update.math_background
        if profile_update.primary_goal is not None:
            profile.__dict__['primary_goal'] = profile_update.primary_goal
        if profile_update.background_questions is not None:
            profile.__dict__['background_questions'] = profile_update.background_questions
        if profile_update.preferences is not None:
            profile.preferences = profile_update.preferences

        # Update timestamps
        profile.updated_at = datetime.now()

        # Save the updated profile
        await self.save_user_profile(profile)

        # Return updated profile as response
        return await self.get_user_profile_response(user_id)

    async def save_user_profile(self, profile: UserProfile) -> bool:
        """
        Save a user profile to storage.
        """
        try:
            profile_data = profile.model_dump()
            self.storage.save_user_profile(profile.id, profile_data)
            return True
        except Exception as e:
            self.logger.error(f"Failed to save user profile: {str(e)}")
            return False

    def verify_password_reset_token(self, token: str) -> Optional[str]:
        """
        Verify a password reset token and return user ID
        """
        if token in self.password_reset_tokens:
            user_id = self.password_reset_tokens[token]
            # Check if token is expired (valid for 1 hour)
            # In a real implementation, we would store and check expiration time
            return user_id
        return None

    def create_password_reset_token(self, user_id: str) -> str:
        """
        Create a password reset token
        """
        token = secrets.token_urlsafe(32)
        self.password_reset_tokens[token] = user_id
        return token

    def reset_user_password(self, token: str, new_password: str) -> bool:
        """
        Reset user password using token
        """
        user_id = self.verify_password_reset_token(token)
        if not user_id:
            return False

        # Find user by user_id
        user_data = None
        for email, u_data in self.users_db.items():
            if u_data["id"] == user_id:
                user_data = u_data
                break

        if not user_data:
            return False

        # Update password
        user_data["hashed_password"] = pwd_context.hash(new_password)
        user_data["updated_at"] = datetime.now()

        # Remove the used token
        del self.password_reset_tokens[token]

        return True

    def validate_email(self, email: str) -> bool:
        """
        Validate email format
        """
        # Basic email validation
        import re
        pattern = r'^[a-zA-Z0-9._%+-]+@[a-zA-Z0-9.-]+\.[a-zA-Z]{2,}$'
        return re.match(pattern, email) is not None

    def execute(self, *args, **kwargs):
        """
        Execute the service operation (required by BaseService).
        """
        # This method should be implemented based on specific user service needs
        raise NotImplementedError("Execute method should be implemented by specific operations")

    def get_user_by_email(self, email: str) -> Optional[Dict[str, Any]]:
        """
        Get user data by email
        """
        return self.users_db.get(email)