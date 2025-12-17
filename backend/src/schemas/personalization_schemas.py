from pydantic import BaseModel
from typing import Optional, List, Dict, Any
from datetime import datetime


class PersonalizationRequest(BaseModel):
    """
    Request schema for content personalization
    """
    user_id: str
    textbook_id: str
    chapter_id: Optional[str] = None
    section_id: Optional[str] = None
    personalization_level: str = "adaptive"  # adaptive, static, none
    content_preferences: Optional[Dict[str, Any]] = None
    learning_style: Optional[str] = None  # visual, auditory, reading, kinesthetic
    difficulty_override: Optional[str] = None  # beginner, intermediate, advanced
    language_preference: Optional[str] = "en"  # Language code


class PersonalizationResponse(BaseModel):
    """
    Response schema for personalized content
    """
    original_content: str
    personalized_content: str
    personalization_applied: bool
    adaptation_reasons: List[str]
    confidence_score: float  # 0-1, how confident the system is in personalization
    processing_time_ms: float
    user_profile_match: float  # 0-1, how well the content matches user profile


class ContentAdaptationRequest(BaseModel):
    """
    Request schema for adapting specific content
    """
    content: str
    user_profile: Dict[str, Any]
    context: Dict[str, Any]  # Additional context like chapter topic, difficulty, etc.
    adaptation_preferences: Optional[Dict[str, Any]] = None


class ContentAdaptationResponse(BaseModel):
    """
    Response schema for adapted content
    """
    original_content: str
    adapted_content: str
    adaptation_type: str  # difficulty, example, explanation, pacing
    adaptation_reason: str
    original_length: int
    adapted_length: int


class PersonalizationPreferencesRequest(BaseModel):
    """
    Request schema for setting personalization preferences
    """
    user_id: str
    learning_style: Optional[str] = None
    content_pacing: Optional[str] = "standard"  # slow, standard, fast, adaptive
    example_preference: Optional[str] = None  # code, math, real_world, theory
    difficulty_tolerance: Optional[float] = 0.7  # 0-1, how much challenge user wants
    feedback_frequency: Optional[str] = "regular"  # none, occasional, regular, frequent
    accessibility_options: Optional[Dict[str, Any]] = None


class PersonalizationPreferencesResponse(BaseModel):
    """
    Response schema for personalization preferences
    """
    user_id: str
    learning_style: Optional[str] = None
    content_pacing: str = "standard"
    example_preference: Optional[str] = None
    difficulty_tolerance: float = 0.7
    feedback_frequency: str = "regular"
    accessibility_options: Optional[Dict[str, Any]] = {}
    updated_at: datetime


class PersonalizationProfile(BaseModel):
    """
    Complete personalization profile combining user profile and preferences
    """
    user_id: str
    user_background: Dict[str, Any]  # From user profile
    learning_preferences: Dict[str, Any]  # From personalization preferences
    content_adaptation_rules: Dict[str, Any]  # Rules for adapting content
    personalization_enabled: bool = True
    last_calculated: datetime
    model_version: str = "1.0"


class AdaptationMetrics(BaseModel):
    """
    Metrics for tracking adaptation effectiveness
    """
    user_id: str
    textbook_id: str
    chapter_id: str
    adaptation_type: str
    original_difficulty: str
    adapted_difficulty: str
    user_engagement_score: float  # 0-1
    comprehension_score: Optional[float] = None  # 0-1
    time_spent_original: Optional[float] = None  # seconds
    time_spent_adapted: Optional[float] = None  # seconds
    feedback_rating: Optional[int] = None  # 1-5
    timestamp: datetime