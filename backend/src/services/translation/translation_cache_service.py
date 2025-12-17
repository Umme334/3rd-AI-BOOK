from typing import Dict, Any, Optional
import logging
from datetime import datetime, timedelta
import hashlib
import asyncio

from ...models.translation_cache import TranslationCache
from ..base_service import BaseService


logger = logging.getLogger(__name__)


class TranslationCacheService(BaseService):
    """
    Service for managing translation caching functionality.
    """

    def __init__(self, max_cache_size: int = 10000, default_ttl_hours: int = 24):
        super().__init__()
        self.translation_cache = {}  # In-memory cache for translations
        self.max_cache_size = max_cache_size
        self.default_ttl_hours = default_ttl_hours

    async def get_cached_translation(
        self,
        original_content: str,
        source_language: str,
        target_language: str
    ) -> Optional[TranslationCache]:
        """
        Get a cached translation if it exists and hasn't expired.
        """
        cache_key = self._generate_cache_key(original_content, source_language, target_language)

        if cache_key in self.translation_cache:
            cached_item = self.translation_cache[cache_key]

            # Check if cache has expired
            if cached_item.expires_at and cached_item.expires_at < datetime.now():
                # Remove expired item
                del self.translation_cache[cache_key]
                logger.info(f"Removed expired translation from cache: {cache_key[:10]}...")
                return None

            logger.info(f"Cache hit for translation: {cache_key[:10]}...")
            return cached_item

        return None

    async def cache_translation(
        self,
        original_content: str,
        source_language: str,
        target_language: str,
        translated_content: str,
        content_type: str = "text"
    ) -> bool:
        """
        Cache a translation with expiration.
        """
        try:
            cache_key = self._generate_cache_key(original_content, source_language, target_language)

            # Check if cache is full, remove oldest entries if needed (simplified LRU)
            if len(self.translation_cache) >= self.max_cache_size:
                # Remove oldest items (in a real implementation, we'd use proper LRU)
                oldest_key = next(iter(self.translation_cache))
                del self.translation_cache[oldest_key]
                logger.info(f"Removed oldest item from cache to make space: {oldest_key[:10]}...")

            cache_item = TranslationCache(
                id=cache_key,
                original_content_id=cache_key,
                original_content=original_content,
                translated_content=translated_content,
                source_language=source_language,
                target_language=target_language,
                content_type=content_type,
                expires_at=datetime.now() + timedelta(hours=self.default_ttl_hours)
            )

            self.translation_cache[cache_key] = cache_item
            logger.info(f"Added translation to cache: {cache_key[:10]}...")
            return True

        except Exception as e:
            self.logger.error(f"Failed to cache translation: {str(e)}")
            return False

    async def is_translation_cached(
        self,
        original_content: str,
        source_language: str,
        target_language: str
    ) -> bool:
        """
        Check if a translation exists in cache and is not expired.
        """
        cached_item = await self.get_cached_translation(
            original_content,
            source_language,
            target_language
        )
        return cached_item is not None

    async def clear_expired_cache(self) -> int:
        """
        Clear expired cache items and return the number of items removed.
        """
        current_time = datetime.now()
        expired_keys = []

        for cache_key, cache_item in self.translation_cache.items():
            if cache_item.expires_at and cache_item.expires_at < current_time:
                expired_keys.append(cache_key)

        for key in expired_keys:
            del self.translation_cache[key]

        logger.info(f"Removed {len(expired_keys)} expired cache items")
        return len(expired_keys)

    async def clear_cache(self):
        """
        Clear the entire translation cache.
        """
        old_size = len(self.translation_cache)
        self.translation_cache.clear()
        logger.info(f"Cleared entire translation cache ({old_size} items removed)")

    async def get_cache_stats(self) -> Dict[str, Any]:
        """
        Get statistics about the translation cache.
        """
        return {
            "cache_size": len(self.translation_cache),
            "max_cache_size": self.max_cache_size,
            "cache_hit_rate": 0.0,  # Would need to track hits separately
            "estimated_memory_usage_mb": len(str(self.translation_cache)) / (1024 * 1024)
        }

    def _generate_cache_key(
        self,
        original_content: str,
        source_language: str,
        target_language: str
    ) -> str:
        """
        Generate a unique cache key for the translation.
        """
        cache_string = f"{original_content}_{source_language}_{target_language}"
        return hashlib.md5(cache_string.encode()).hexdigest()

    async def get_cache_size(self) -> int:
        """
        Get the current number of cached translations.
        """
        return len(self.translation_cache)

    async def remove_translation(
        self,
        original_content: str,
        source_language: str,
        target_language: str
    ) -> bool:
        """
        Remove a specific translation from the cache.
        """
        cache_key = self._generate_cache_key(original_content, source_language, target_language)

        if cache_key in self.translation_cache:
            del self.translation_cache[cache_key]
            logger.info(f"Removed translation from cache: {cache_key[:10]}...")
            return True

        return False

    async def get_all_cached_translations(self) -> Dict[str, TranslationCache]:
        """
        Get all cached translations (use with caution for large caches).
        """
        # Remove expired items first
        await self.clear_expired_cache()
        return self.translation_cache.copy()

    async def update_cache_ttl(
        self,
        original_content: str,
        source_language: str,
        target_language: str,
        new_ttl_hours: int
    ) -> bool:
        """
        Update the TTL for a specific cached translation.
        """
        cache_key = self._generate_cache_key(original_content, source_language, target_language)

        if cache_key in self.translation_cache:
            cached_item = self.translation_cache[cache_key]
            cached_item.expires_at = datetime.now() + timedelta(hours=new_ttl_hours)
            return True

        return False

    def execute(self, *args, **kwargs):
        """
        Execute the service operation (required by BaseService).
        """
        # This method should be implemented based on specific cache needs
        raise NotImplementedError("Execute method should be implemented by specific operations")