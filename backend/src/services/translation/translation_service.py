from typing import Dict, Any, List, Optional
from datetime import datetime, timedelta
import logging
import asyncio

from ...models.translation_cache import TranslationCache
from ...models.textbook import Textbook
from ...models.chapter import Chapter
from ...models.section import Section
from ...services.openai_client import OpenAIClient
from ...schemas.translation_schemas import (
    TranslationRequest,
    TranslationResponse,
    TranslationBatchRequest,
    TranslationBatchResponse,
    TranslationMetrics
)
from ..auth.user_service import UserService
from ..auth.background_capture_service import BackgroundCaptureService
from .urdu_translator import UrduTranslator
from .translation_cache_service import TranslationCacheService
from ..base_service import BaseService


logger = logging.getLogger(__name__)


class TranslationService(BaseService):
    """
    Service for translating content to Urdu and other languages.
    """

    def __init__(self):
        super().__init__()
        self.openai_client = OpenAIClient()
        self.urdu_translator = UrduTranslator()
        self.user_service = UserService()
        self.background_service = BackgroundCaptureService()
        self.translation_cache_service = TranslationCacheService()
        self.translation_metrics = {}  # Store metrics per user/textbook

    async def translate_content(
        self,
        request: TranslationRequest
    ) -> TranslationResponse:
        """
        Translate content from source to target language
        """
        try:
            start_time = datetime.now()

            # Check if translation exists in cache
            cached_result = await self.translation_cache_service.get_cached_translation(
                request.content,
                request.source_language,
                request.target_language
            )

            if cached_result:
                logger.info(f"Cache hit for translation: {cached_result.id[:10]}...")
                return TranslationResponse(
                    original_content=request.content,
                    translated_content=cached_result.translated_content,
                    source_language=request.source_language,
                    target_language=request.target_language,
                    content_type=request.content_type,
                    translation_quality_score=0.9,  # Assuming high quality for cached translations
                    processing_time_ms=(datetime.now() - start_time).total_seconds() * 1000,
                    translation_cache_hit=True,
                    translated_at=datetime.now()
                )

            # Perform translation
            translated_content = await self._perform_translation(
                request.content,
                request.source_language,
                request.target_language,
                request.translation_level
            )

            # Calculate quality score (simplified)
            quality_score = self._calculate_translation_quality(
                request.content,
                translated_content
            )

            # Cache the result if it's not too large
            if len(translated_content) < 10000:  # Don't cache very large translations
                await self.translation_cache_service.cache_translation(
                    request.content,
                    request.source_language,
                    request.target_language,
                    translated_content,
                    request.content_type
                )

            processing_time = (datetime.now() - start_time).total_seconds() * 1000

            # Record metrics
            await self._record_translation_metrics(
                request.user_id,
                request.textbook_id,
                len(request.content),
                quality_score,
                processing_time
            )

            response = TranslationResponse(
                original_content=request.content,
                translated_content=translated_content,
                source_language=request.source_language,
                target_language=request.target_language,
                content_type=request.content_type,
                translation_quality_score=quality_score,
                processing_time_ms=processing_time,
                translation_cache_hit=False,
                translated_at=datetime.now()
            )

            return response

        except Exception as e:
            logger.error(f"Error translating content: {str(e)}")
            raise

    async def translate_textbook(self, textbook: Textbook, target_language: str = "ur") -> Textbook:
        """
        Translate textbook content to the target language.
        """
        if target_language not in textbook.available_translations:
            textbook.available_translations.append(target_language)

        # Translate each chapter
        for chapter in textbook.chapters:
            await self.translate_chapter(chapter, target_language)

        return textbook

    async def translate_chapter(self, chapter: Chapter, target_language: str = "ur") -> Chapter:
        """
        Translate chapter content to the target language.
        """
        # Check if translation exists in cache
        cached_translation = await self.translation_cache_service.get_cached_translation(
            chapter.title,
            "en",  # Assuming source is English
            target_language
        )

        if cached_translation:
            chapter.translated_titles[target_language] = cached_translation.translated_content
        else:
            # Translate the title
            translated_title = await self._translate_text(chapter.title, target_language)
            chapter.translated_titles[target_language] = translated_title

            # Store in cache
            await self.translation_cache_service.cache_translation(
                original_content=chapter.title,
                source_language="en",
                target_language=target_language,
                translated_content=translated_title,
                content_type="chapter"
            )

        return chapter

    async def translate_section(self, section: Section, target_language: str = "ur") -> Section:
        """
        Translate section content to the target language.
        """
        # Check if translation exists in cache
        cached_translation = await self.translation_cache_service.get_cached_translation(
            section.content,
            "en",  # Assuming source is English
            target_language
        )

        if cached_translation:
            section.translated_content[target_language] = cached_translation.translated_content
        else:
            # Translate the content
            translated_content = await self._translate_text(section.content, target_language)
            section.translated_content[target_language] = translated_content

            # Store in cache
            await self.translation_cache_service.cache_translation(
                original_content=section.content,
                source_language="en",
                target_language=target_language,
                translated_content=translated_content,
                content_type="section"
            )

        return section

    async def translate_interactive_element(self, element: Dict[str, Any], target_language: str = "ur") -> Dict[str, Any]:
        """
        Translate interactive element content to the target language.
        """
        content = element.get('content', '')

        # Check if translation exists in cache
        cached_translation = await self.translation_cache_service.get_cached_translation(
            content,
            "en",  # Assuming source is English
            target_language
        )

        if cached_translation:
            element['translated_content'] = {target_language: cached_translation.translated_content}
        else:
            # Translate the content
            translated_content = await self._translate_text(content, target_language)
            element['translated_content'] = {target_language: translated_content}

            # Store in cache
            await self.translation_cache_service.cache_translation(
                original_content=content,
                source_language="en",
                target_language=target_language,
                translated_content=translated_content,
                content_type="interactive_element"
            )

        return element

    async def _perform_translation(
        self,
        content: str,
        source_language: str,
        target_language: str,
        translation_level: str = "literal"
    ) -> str:
        """
        Perform the actual translation based on language pair
        """
        if target_language.lower() == "ur" or target_language.lower() == "urdu":
            # Use Urdu translator for Urdu translations
            return await self.urdu_translator.translate(
                content,
                source_language,
                translation_level
            )
        else:
            # For other languages, use OpenAI
            return await self._translate_text(content, target_language)

    async def _translate_text(self, text: str, target_language: str = "ur") -> str:
        """
        Translate text to the target language using OpenAI.
        """
        # For Urdu, we'll use a specific translation prompt
        if target_language.lower() == "ur" or target_language.lower() == "urdu":
            translated_text = self.openai_client.translate_to_urdu(text)
        else:
            translated_text = self.openai_client.translate_text(text, target_language)

        return translated_text

    def _calculate_translation_quality(
        self,
        original_content: str,
        translated_content: str
    ) -> float:
        """
        Calculate a simple quality score for the translation
        """
        # This is a simplified quality calculation
        # In a real implementation, this would be more sophisticated
        if len(translated_content) == 0:
            return 0.0

        # Check if translation is reasonably close in length to original
        length_ratio = len(translated_content) / len(original_content)
        if 0.5 <= length_ratio <= 2.0:
            return 0.8  # Good quality if length is reasonable
        else:
            return 0.5  # Lower quality if length is way off

    async def _record_translation_metrics(
        self,
        user_id: Optional[str],
        textbook_id: Optional[str],
        content_length: int,
        quality_score: float,
        processing_time: float
    ):
        """
        Record translation metrics for analytics
        """
        try:
            # Create a unique key for this user/textbook combination
            key = f"{user_id}_{textbook_id}" if user_id and textbook_id else "anonymous"

            if key not in self.translation_metrics:
                self.translation_metrics[key] = {
                    "total_translations": 0,
                    "total_characters": 0,
                    "total_quality_score": 0.0,
                    "total_processing_time": 0.0,
                    "last_updated": datetime.now()
                }

            # Update metrics
            metrics = self.translation_metrics[key]
            metrics["total_translations"] += 1
            metrics["total_characters"] += content_length
            metrics["total_quality_score"] += quality_score
            metrics["total_processing_time"] += processing_time
            metrics["last_updated"] = datetime.now()

        except Exception as e:
            logger.error(f"Error recording translation metrics: {str(e)}")

    def get_translation_metrics(
        self,
        user_id: Optional[str] = None,
        textbook_id: Optional[str] = None
    ) -> Optional[TranslationMetrics]:
        """
        Get translation metrics for a user or textbook
        """
        key = f"{user_id}_{textbook_id}" if user_id and textbook_id else "anonymous"

        if key in self.translation_metrics:
            metrics = self.translation_metrics[key]
            avg_quality = metrics["total_quality_score"] / metrics["total_translations"] if metrics["total_translations"] > 0 else 0.0

            return TranslationMetrics(
                user_id=user_id,
                textbook_id=textbook_id,
                total_translations=metrics["total_translations"],
                cache_hit_rate=0.0,  # Would be calculated if we tracked cache hits separately
                average_quality_score=avg_quality,
                total_characters_translated=metrics["total_characters"],
                timestamp=metrics["last_updated"]
            )

        return None

    async def translate_batch(
        self,
        batch_request: TranslationBatchRequest
    ) -> TranslationBatchResponse:
        """
        Translate multiple pieces of content in batch
        """
        try:
            start_time = datetime.now()
            translations = []
            cache_hits = 0

            for content in batch_request.contents:
                request = TranslationRequest(
                    content=content,
                    source_language=batch_request.source_language,
                    target_language=batch_request.target_language,
                    preserve_formatting=batch_request.preserve_formatting
                )
                response = await self.translate_content(request)

                if response.translation_cache_hit:
                    cache_hits += 1

                translations.append(response)

            total_time = (datetime.now() - start_time).total_seconds() * 1000

            return TranslationBatchResponse(
                translations=translations,
                total_processed=len(batch_request.contents),
                cache_hits=cache_hits,
                processing_time_ms=total_time
            )
        except Exception as e:
            logger.error(f"Error in batch translation: {str(e)}")
            raise

    async def get_translation_status(self, content: str, target_language: str) -> Dict[str, Any]:
        """
        Get the translation status for specific content.
        """
        cached_item = await self.translation_cache_service.get_cached_translation(
            content,
            "en",  # Assuming source is English
            target_language
        )

        if cached_item:
            return {
                "status": "completed",
                "translated_content_available": True,
                "expires_at": cached_item.expires_at
            }
        else:
            return {
                "status": "not_cached",
                "translated_content_available": False
            }

    async def clear_expired_cache(self) -> int:
        """
        Clear expired cache items and return the number of items removed.
        """
        return await self.translation_cache_service.clear_expired_cache()

    async def clear_cache(self):
        """
        Clear the translation cache
        """
        await self.translation_cache_service.clear_cache()

    async def get_cache_stats(self) -> Dict[str, Any]:
        """
        Get statistics about the translation cache
        """
        return await self.translation_cache_service.get_cache_stats()

    async def check_translation_support(
        self,
        source_language: str,
        target_language: str
    ) -> bool:
        """
        Check if translation is supported between these languages
        """
        # Currently only Urdu translation is supported
        return target_language.lower() in ["ur", "urdu"]

    def execute(self, *args, **kwargs):
        """
        Execute the service operation (required by BaseService).
        """
        # This method should be implemented based on specific translation needs
        raise NotImplementedError("Execute method should be implemented by specific operations")