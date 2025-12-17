from pydantic import BaseModel, EmailStr, validator
from typing import Optional, List, Dict, Any
from datetime import datetime


class AuthRequest(BaseModel):
    """
    Base request schema for authentication operations
    """
    email: EmailStr
    password: str


class AuthResponse(BaseModel):
    """
    Response schema for authentication operations
    """
    user_id: str
    email: EmailStr
    access_token: str
    refresh_token: Optional[str] = None
    token_type: str = "bearer"
    expires_at: datetime
    user_profile: Optional[Dict[str, Any]] = None


class SignupRequest(BaseModel):
    """
    Request schema for user registration
    """
    email: EmailStr
    password: str
    first_name: str
    last_name: Optional[str] = None
    # Background information fields for Physical AI & Humanoid Robotics course
    software_experience: Optional[str] = None  # beginner, intermediate, advanced
    hardware_experience: Optional[str] = None  # beginner, intermediate, advanced
    programming_languages: Optional[List[str]] = []
    hardware_platforms: Optional[List[str]] = []
    robotics_experience: Optional[str] = None  # none, basic, intermediate, advanced
    math_background: Optional[str] = None  # basic, intermediate, advanced
    primary_goal: Optional[str] = None  # education, research, hobby, career
    background_questions: Optional[Dict[str, Any]] = {}


class SignupResponse(BaseModel):
    """
    Response schema for user registration
    """
    user_id: str
    email: EmailStr
    access_token: str
    token_type: str = "bearer"
    profile_created: bool
    message: str = "User registered successfully"


class SigninRequest(BaseModel):
    """
    Request schema for user login
    """
    email: EmailStr
    password: str


class SigninResponse(BaseModel):
    """
    Response schema for user login
    """
    user_id: str
    email: EmailStr
    access_token: str
    refresh_token: Optional[str] = None
    token_type: str = "bearer"
    expires_at: datetime
    user_profile: Optional[Dict[str, Any]] = None
    message: str = "Login successful"


class UserProfileUpdateRequest(BaseModel):
    """
    Request schema for updating user profile
    """
    first_name: Optional[str] = None
    last_name: Optional[str] = None
    # Background information fields that can be updated
    software_experience: Optional[str] = None
    hardware_experience: Optional[str] = None
    programming_languages: Optional[List[str]] = None
    hardware_platforms: Optional[List[str]] = None
    robotics_experience: Optional[str] = None
    math_background: Optional[str] = None
    primary_goal: Optional[str] = None
    background_questions: Optional[Dict[str, Any]] = None
    preferences: Optional[Dict[str, Any]] = None  # Personalization preferences


class UserProfileResponse(BaseModel):
    """
    Response schema for user profile
    """
    user_id: str
    email: EmailStr
    first_name: str
    last_name: Optional[str] = None
    created_at: datetime
    updated_at: datetime
    # Background information
    software_experience: Optional[str] = None
    hardware_experience: Optional[str] = None
    programming_languages: Optional[List[str]] = []
    hardware_platforms: Optional[List[str]] = []
    robotics_experience: Optional[str] = None
    math_background: Optional[str] = None
    primary_goal: Optional[str] = None
    background_questions: Optional[Dict[str, Any]] = {}
    preferences: Optional[Dict[str, Any]] = {}
    # Profile status
    profile_completed: bool = False
    last_login: Optional[datetime] = None


class TokenRefreshRequest(BaseModel):
    """
    Request schema for token refresh
    """
    refresh_token: str


class TokenRefreshResponse(BaseModel):
    """
    Response schema for token refresh
    """
    access_token: str
    refresh_token: str
    token_type: str = "bearer"
    expires_at: datetime


class PasswordResetRequest(BaseModel):
    """
    Request schema for password reset
    """
    email: EmailStr


class PasswordResetResponse(BaseModel):
    """
    Response schema for password reset request
    """
    message: str = "Password reset email sent if user exists"


class PasswordResetConfirmRequest(BaseModel):
    """
    Request schema for confirming password reset
    """
    token: str
    new_password: str


class AuthValidationResponse(BaseModel):
    """
    Response schema for authentication validation
    """
    authenticated: bool
    user_id: Optional[str] = None
    email: Optional[EmailStr] = None
    message: str = "Authentication validated"