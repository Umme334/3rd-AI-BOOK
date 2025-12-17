from pydantic import BaseModel
from typing import Optional, List, Dict, Any
from datetime import datetime


class TranslationRequest(BaseModel):
    """
    Request schema for content translation
    """
    content: str
    source_language: str = "en"  # Default source is English
    target_language: str = "ur"  # Default target is Urdu
    content_type: str = "text"  # text, chapter, section, textbook
    textbook_id: Optional[str] = None
    user_id: Optional[str] = None
    preserve_formatting: bool = True  # Whether to preserve original formatting
    translation_level: str = "literal"  # literal, contextual, adaptive


class TranslationResponse(BaseModel):
    """
    Response schema for content translation
    """
    original_content: str
    translated_content: str
    source_language: str
    target_language: str
    content_type: str
    translation_quality_score: float  # 0-1, how confident the system is in translation
    processing_time_ms: float
    translation_cache_hit: bool  # Whether result came from cache
    translated_at: datetime


class TranslationBatchRequest(BaseModel):
    """
    Request schema for batch content translation
    """
    contents: List[str]
    source_language: str = "en"
    target_language: str = "ur"
    preserve_formatting: bool = True


class TranslationBatchResponse(BaseModel):
    """
    Response schema for batch content translation
    """
    translations: List[TranslationResponse]
    total_processed: int
    cache_hits: int
    processing_time_ms: float


class TranslationCacheRequest(BaseModel):
    """
    Request schema for translation cache operations
    """
    source_text: str
    source_language: str
    target_language: str


class TranslationCacheResponse(BaseModel):
    """
    Response schema for translation cache operations
    """
    cached: bool
    translated_text: Optional[str] = None
    cache_key: str
    expires_at: Optional[datetime] = None


class TranslationLanguageSupport(BaseModel):
    """
    Schema for translation language support information
    """
    source_language: str
    target_language: str
    supported: bool
    translation_quality: str  # high, medium, low
    features_supported: List[str]  # List of features available for this language pair


class TranslationMetrics(BaseModel):
    """
    Schema for translation metrics and statistics
    """
    user_id: Optional[str] = None
    textbook_id: Optional[str] = None
    chapter_id: Optional[str] = None
    total_translations: int
    cache_hit_rate: float
    average_quality_score: float
    total_characters_translated: int
    timestamp: datetime


class TranslationPreferenceRequest(BaseModel):
    """
    Request schema for user translation preferences
    """
    user_id: str
    preferred_source_language: str = "en"
    preferred_target_language: str = "ur"
    translation_style: str = "formal"  # formal, informal, academic
    preferred_dialect: Optional[str] = None  # For languages with dialects
    auto_translate_new_content: bool = True


class TranslationPreferenceResponse(BaseModel):
    """
    Response schema for user translation preferences
    """
    user_id: str
    preferred_source_language: str
    preferred_target_language: str
    translation_style: str
    preferred_dialect: Optional[str]
    auto_translate_new_content: bool
    updated_at: datetime