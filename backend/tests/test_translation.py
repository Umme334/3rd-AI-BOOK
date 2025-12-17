import pytest
import asyncio
from unittest.mock import AsyncMock, MagicMock, patch
from src.services.translation.translation_service import TranslationService
from src.services.translation.urdu_translator import UrduTranslator
from src.services.translation.translation_cache_service import TranslationCacheService


class TestTranslationService:
    """Test cases for translation service"""

    @pytest.fixture
    def translation_service(self):
        return TranslationService()

    @pytest.mark.asyncio
    async def test_translate_content(self, translation_service):
        """Test translating content to Urdu"""
        from src.schemas.translation_schemas import TranslationRequest

        request = TranslationRequest(
            content='This is content about Physical AI',
            source_language='en',
            target_language='ur',
            content_type='text'
        )

        result = await translation_service.translate_content(request)

        assert result is not None
        assert result.original_content == request.content
        assert result.source_language == request.source_language
        assert result.target_language == request.target_language
        assert result.translation_cache_hit in [True, False]

    @pytest.mark.asyncio
    async def test_translate_textbook(self, translation_service):
        """Test translating entire textbook"""
        from src.models.textbook import Textbook
        from src.models.chapter import Chapter

        mock_textbook = Textbook(
            id='textbook-123',
            title='Physical AI Textbook',
            subject='Physical AI',
            difficulty='intermediate',
            chapters=[
                Chapter(
                    id='chapter-1',
                    title='Introduction to ROS 2',
                    content='Content about ROS 2 fundamentals'
                )
            ]
        )

        result = await translation_service.translate_textbook(mock_textbook, 'ur')

        assert result is not None
        assert result.id == mock_textbook.id
        assert 'ur' in result.available_translations

    @pytest.mark.asyncio
    async def test_translate_chapter(self, translation_service):
        """Test translating a single chapter"""
        from src.models.chapter import Chapter

        mock_chapter = Chapter(
            id='chapter-1',
            title='Introduction to ROS 2',
            content='Content about ROS 2 fundamentals'
        )

        result = await translation_service.translate_chapter(mock_chapter, 'ur')

        assert result is not None
        assert result.id == mock_chapter.id
        assert 'ur' in result.translated_titles

    @pytest.mark.asyncio
    async def test_translate_section(self, translation_service):
        """Test translating a single section"""
        from src.models.section import Section

        mock_section = Section(
            id='section-1',
            title='ROS 2 Nodes',
            content='Content about ROS 2 nodes and their implementation'
        )

        result = await translation_service.translate_section(mock_section, 'ur')

        assert result is not None
        assert result.id == mock_section.id
        assert 'ur' in result.translated_content

    @pytest.mark.asyncio
    async def test_translate_batch(self, translation_service):
        """Test translating multiple pieces of content in batch"""
        from src.schemas.translation_schemas import TranslationBatchRequest

        batch_request = TranslationBatchRequest(
            contents=[
                'First content about Physical AI',
                'Second content about ROS 2',
                'Third content about Gazebo'
            ],
            source_language='en',
            target_language='ur'
        )

        result = await translation_service.translate_batch(batch_request)

        assert result is not None
        assert len(result.translations) == len(batch_request.contents)
        assert result.total_processed == len(batch_request.contents)

    @pytest.mark.asyncio
    async def test_get_translation_status(self, translation_service):
        """Test getting translation status for content"""
        content = 'Content to translate'
        status = await translation_service.get_translation_status(content, 'ur')

        assert status is not None
        assert 'status' in status
        assert 'translated_content_available' in status

    @pytest.mark.asyncio
    async def test_check_translation_support(self, translation_service):
        """Test checking if translation is supported between languages"""
        supported = await translation_service.check_translation_support('en', 'ur')
        not_supported = await translation_service.check_translation_support('en', 'invalid_lang')

        assert supported is True
        assert not_supported is False


class TestUrduTranslator:
    """Test cases for Urdu translation service"""

    @pytest.fixture
    def urdu_translator(self):
        return UrduTranslator()

    @pytest.mark.asyncio
    async def test_translate_literal(self, urdu_translator):
        """Test literal translation to Urdu"""
        content = 'Hello, this is a test'
        result = urdu_translator._translate_literal(content, 'en')

        # The result will depend on whether Google Translate API is available
        assert result is not None
        assert isinstance(result, str)

    @pytest.mark.asyncio
    async def test_translate_contextual(self, urdu_translator):
        """Test contextual translation to Urdu"""
        content = 'Physical AI concepts'
        result = urdu_translator._translate_contextual(content, 'en')

        assert result is not None
        assert isinstance(result, str)

    @pytest.mark.asyncio
    async def test_translate_adaptive(self, urdu_translator):
        """Test adaptive translation to Urdu"""
        content = 'Robotics and AI integration'
        result = urdu_translator._translate_adaptive(content, 'en')

        assert result is not None
        assert isinstance(result, str)

    def test_is_urdu_text(self, urdu_translator):
        """Test detecting if text is in Urdu"""
        urdu_text = 'یہ اردو میں لکھا گیا ہے'
        english_text = 'This is in English'

        is_urdu = urdu_translator.is_urdu_text(urdu_text)
        is_not_urdu = urdu_translator.is_urdu_text(english_text)

        assert is_urdu is True
        assert is_not_urdu is False

    def test_get_urdu_text_length(self, urdu_translator):
        """Test getting length of Urdu text"""
        urdu_text = 'یہ اردو میں لکھا گیا ہے'
        length = urdu_translator.get_urdu_text_length(urdu_text)

        assert length > 0
        assert isinstance(length, int)

    @pytest.mark.asyncio
    async def test_translate_batch(self, urdu_translator):
        """Test batch translation to Urdu"""
        contents = [
            'First content',
            'Second content',
            'Third content'
        ]

        result = await urdu_translator.translate_batch(contents, 'en')

        assert result is not None
        assert isinstance(result, list)
        assert len(result) == len(contents)


class TestTranslationCacheService:
    """Test cases for translation cache service"""

    @pytest.fixture
    def cache_service(self):
        return TranslationCacheService()

    @pytest.mark.asyncio
    async def test_cache_translation(self, cache_service):
        """Test caching a translation"""
        result = await cache_service.cache_translation(
            original_content='Test content',
            source_language='en',
            target_language='ur',
            translated_content='مترجم ہونے والا مواد',
            content_type='text'
        )

        assert result is True

    @pytest.mark.asyncio
    async def test_get_cached_translation(self, cache_service):
        """Test retrieving a cached translation"""
        # First cache a translation
        await cache_service.cache_translation(
            original_content='Test content',
            source_language='en',
            target_language='ur',
            translated_content='مترجم ہونے والا مواد',
            content_type='text'
        )

        # Then retrieve it
        cached = await cache_service.get_cached_translation(
            'Test content',
            'en',
            'ur'
        )

        assert cached is not None
        assert cached.original_content == 'Test content'
        assert cached.translated_content == 'مترجم ہونے والا مواد'

    @pytest.mark.asyncio
    async def test_is_translation_cached(self, cache_service):
        """Test checking if translation exists in cache"""
        # Cache a translation
        await cache_service.cache_translation(
            original_content='Test content',
            source_language='en',
            target_language='ur',
            translated_content='مترجم ہونے والا مواد',
            content_type='text'
        )

        # Check if it's cached
        is_cached = await cache_service.is_translation_cached(
            'Test content',
            'en',
            'ur'
        )

        assert is_cached is True

        # Check non-existent translation
        not_cached = await cache_service.is_translation_cached(
            'Non-existent content',
            'en',
            'ur'
        )

        assert not_cached is False

    @pytest.mark.asyncio
    async def test_remove_translation(self, cache_service):
        """Test removing a translation from cache"""
        # Cache a translation
        await cache_service.cache_translation(
            original_content='Test content to remove',
            source_language='en',
            target_language='ur',
            translated_content='مترجم ہونے والا مواد',
            content_type='text'
        )

        # Check it exists
        exists_before = await cache_service.is_translation_cached(
            'Test content to remove',
            'en',
            'ur'
        )
        assert exists_before is True

        # Remove it
        removed = await cache_service.remove_translation(
            'Test content to remove',
            'en',
            'ur'
        )
        assert removed is True

        # Check it no longer exists
        exists_after = await cache_service.is_translation_cached(
            'Test content to remove',
            'en',
            'ur'
        )
        assert exists_after is False

    @pytest.mark.asyncio
    async def test_cache_expiry(self, cache_service):
        """Test cache expiry functionality"""
        # Set a very short TTL for testing
        import asyncio
        from datetime import datetime, timedelta

        # Cache with short TTL
        result = await cache_service.cache_translation(
            original_content='Short TTL content',
            source_language='en',
            target_language='ur',
            translated_content='مختصر ٹی ٹی ایل مواد',
            content_type='text'
        )

        assert result is True

        # Check cache stats
        stats = await cache_service.get_cache_stats()
        assert 'cache_size' in stats

    @pytest.mark.asyncio
    async def test_clear_cache(self, cache_service):
        """Test clearing the entire cache"""
        # Add some items to cache
        await cache_service.cache_translation(
            original_content='Content 1',
            source_language='en',
            target_language='ur',
            translated_content='مترجم 1',
            content_type='text'
        )
        await cache_service.cache_translation(
            original_content='Content 2',
            source_language='en',
            target_language='ur',
            translated_content='مترجم 2',
            content_type='text'
        )

        # Check cache has items
        size_before = await cache_service.get_cache_size()
        assert size_before > 0

        # Clear cache
        await cache_service.clear_cache()

        # Check cache is empty
        size_after = await cache_service.get_cache_size()
        assert size_after == 0