from typing import Any, Dict, Optional, Tuple
from datetime import datetime, timedelta
import hashlib
import asyncio
from collections import OrderedDict
from ..utils.logger import logger


class CacheMiddleware:
    """
    Caching middleware for translation and personalization services
    """

    def __init__(self, max_size: int = 1000, default_ttl_minutes: int = 60):
        self.cache: OrderedDict[str, Tuple[Any, datetime]] = OrderedDict()
        self.max_size = max_size
        self.default_ttl_minutes = default_ttl_minutes

    def _generate_cache_key(self, *args, **kwargs) -> str:
        """
        Generate a unique cache key from arguments
        """
        cache_string = f"{args}_{sorted(kwargs.items())}"
        return hashlib.md5(str(cache_string).encode()).hexdigest()

    def _is_expired(self, expiry_time: datetime) -> bool:
        """
        Check if cache entry has expired
        """
        return datetime.now() > expiry_time

    def _cleanup_expired(self):
        """
        Remove expired entries from cache
        """
        expired_keys = []
        for key, (value, expiry_time) in self.cache.items():
            if self._is_expired(expiry_time):
                expired_keys.append(key)

        for key in expired_keys:
            del self.cache[key]

    async def get(self, key: str) -> Optional[Any]:
        """
        Get value from cache
        """
        self._cleanup_expired()

        if key in self.cache:
            value, expiry_time = self.cache[key]
            if not self._is_expired(expiry_time):
                logger.debug(f"Cache hit for key: {key[:10]}...")
                return value
            else:
                # Remove expired entry
                del self.cache[key]

        logger.debug(f"Cache miss for key: {key[:10]}...")
        return None

    async def set(self, key: str, value: Any, ttl_minutes: Optional[int] = None) -> bool:
        """
        Set value in cache with TTL
        """
        if ttl_minutes is None:
            ttl_minutes = self.default_ttl_minutes

        expiry_time = datetime.now() + timedelta(minutes=ttl_minutes)

        # Remove oldest entries if cache is full
        while len(self.cache) >= self.max_size:
            oldest_key = next(iter(self.cache))
            del self.cache[oldest_key]

        self.cache[key] = (value, expiry_time)
        logger.debug(f"Cache set for key: {key[:10]}...")

        return True

    async def invalidate(self, key: str) -> bool:
        """
        Remove specific key from cache
        """
        if key in self.cache:
            del self.cache[key]
            logger.debug(f"Cache invalidated for key: {key[:10]}...")
            return True
        return False

    async def invalidate_pattern(self, pattern: str) -> int:
        """
        Remove keys matching a pattern from cache
        """
        keys_to_remove = []
        for key in self.cache:
            if pattern in key:
                keys_to_remove.append(key)

        for key in keys_to_remove:
            del self.cache[key]

        logger.debug(f"Cache invalidated {len(keys_to_remove)} keys matching pattern: {pattern}")
        return len(keys_to_remove)

    async def clear(self):
        """
        Clear entire cache
        """
        count = len(self.cache)
        self.cache.clear()
        logger.debug(f"Cleared entire cache ({count} items removed)")
        return count

    async def get_cache_stats(self) -> Dict[str, Any]:
        """
        Get cache statistics
        """
        self._cleanup_expired()
        return {
            "size": len(self.cache),
            "max_size": self.max_size,
            "utilization": len(self.cache) / self.max_size if self.max_size > 0 else 0,
            "default_ttl_minutes": self.default_ttl_minutes
        }

    # Specific caching methods for translation and personalization
    async def get_translation(self, source_text: str, source_lang: str, target_lang: str) -> Optional[str]:
        """
        Get cached translation
        """
        key = self._generate_cache_key("translation", source_text, source_lang, target_lang)
        return await self.get(key)

    async def cache_translation(self, source_text: str, source_lang: str, target_lang: str, translated_text: str, ttl_minutes: Optional[int] = None) -> bool:
        """
        Cache translation result
        """
        key = self._generate_cache_key("translation", source_text, source_lang, target_lang)
        return await self.set(key, translated_text, ttl_minutes)

    async def get_personalized_content(self, original_content: str, user_profile_id: str) -> Optional[str]:
        """
        Get cached personalized content
        """
        key = self._generate_cache_key("personalization", original_content, user_profile_id)
        return await self.get(key)

    async def cache_personalized_content(self, original_content: str, user_profile_id: str, personalized_content: str, ttl_minutes: Optional[int] = None) -> bool:
        """
        Cache personalized content result
        """
        key = self._generate_cache_key("personalization", original_content, user_profile_id)
        return await self.set(key, personalized_content, ttl_minutes)


# Global instance
cache_middleware = CacheMiddleware(max_size=1000, default_ttl_minutes=60)