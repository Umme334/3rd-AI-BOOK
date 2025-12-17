import pytest
import asyncio
from unittest.mock import Mock, AsyncMock, patch
from src.services.textbook_service import TextbookService
from src.services.chatbot.rag_service import RAGService
from src.services.translation.translation_service import TranslationService
from src.services.personalization.personalization_service import PersonalizationService
from src.models.textbook import Textbook


class TestTextbookService:
    """
    Unit tests for TextbookService
    """

    @pytest.fixture
    def textbook_service(self):
        return TextbookService()

    @pytest.mark.asyncio
    async def test_create_textbook(self, textbook_service):
        """
        Test creating a textbook
        """
        request_data = {
            "title": "Test Textbook",
            "subject": "Physical AI",
            "difficulty": "beginner",
            "target_audience": "students",
            "chapter_count": 5
        }

        textbook = await textbook_service.create_textbook(request_data)

        assert isinstance(textbook, Textbook)
        assert textbook.title == "Test Textbook"
        assert textbook.subject == "Physical AI"
        assert len(textbook.chapters) == 0  # Initially no chapters

    @pytest.mark.asyncio
    async def test_get_textbook(self, textbook_service):
        """
        Test getting a textbook
        """
        # Create a textbook first
        request_data = {
            "title": "Test Textbook",
            "subject": "Physical AI",
            "difficulty": "beginner",
            "target_audience": "students",
            "chapter_count": 5
        }

        created_textbook = await textbook_service.create_textbook(request_data)
        retrieved_textbook = textbook_service.get_textbook(created_textbook.id)

        assert retrieved_textbook is not None
        assert retrieved_textbook.id == created_textbook.id
        assert retrieved_textbook.title == created_textbook.title


class TestRAGService:
    """
    Unit tests for RAGService
    """

    @pytest.fixture
    def rag_service(self):
        return RAGService()

    @pytest.mark.asyncio
    async def test_index_textbook(self, rag_service):
        """
        Test indexing a textbook
        """
        # Create a mock textbook
        textbook = Textbook(
            id="test_id",
            title="Test Textbook",
            subject="Physical AI",
            difficulty="beginner",
            target_audience="students"
        )

        # Mock the embedding service
        with patch.object(rag_service, '_embed_text', new_callable=AsyncMock) as mock_embed:
            mock_embed.return_value = [0.1, 0.2, 0.3]  # Mock embedding

            result = await rag_service.index_textbook(textbook)

            assert result is True
            mock_embed.assert_called()

    @pytest.mark.asyncio
    async def test_query_textbook(self, rag_service):
        """
        Test querying a textbook
        """
        query = "What is embodied intelligence?"

        # Mock the search functionality
        with patch.object(rag_service, '_search_similar_content', new_callable=AsyncMock) as mock_search:
            mock_search.return_value = ["Result 1", "Result 2"]

            results = await rag_service.query_textbook("test_id", query)

            assert len(results) == 2
            mock_search.assert_called()


class TestTranslationService:
    """
    Unit tests for TranslationService
    """

    @pytest.fixture
    def translation_service(self):
        return TranslationService()

    @pytest.mark.asyncio
    async def test_translate_content(self, translation_service):
        """
        Test translating content
        """
        from src.schemas.translation_schemas import TranslationRequest

        request = TranslationRequest(
            content="Hello, world!",
            source_language="en",
            target_language="ur"
        )

        # Mock the translation functionality
        with patch.object(translation_service, '_perform_translation', new_callable=AsyncMock) as mock_translate:
            mock_translate.return_value = "ہیلو، دنیا!"

            response = await translation_service.translate_content(request)

            assert response.translated_content == "ہیло، دنیا!"
            assert response.source_language == "en"
            assert response.target_language == "ur"


class TestPersonalizationService:
    """
    Unit tests for PersonalizationService
    """

    @pytest.fixture
    def personalization_service(self):
        return PersonalizationService()

    @pytest.mark.asyncio
    async def test_adapt_content(self, personalization_service):
        """
        Test adapting content
        """
        from src.schemas.personalization_schemas import PersonalizationRequest

        request = PersonalizationRequest(
            content="This is a complex algorithm.",
            user_id="test_user",
            content_type="text"
        )

        # Mock the adaptation functionality
        with patch.object(personalization_service, '_adapt_content_for_user', new_callable=AsyncMock) as mock_adapt:
            mock_adapt.return_value = "This is a simple method."

            response = await personalization_service.adapt_content(request)

            assert response.adapted_content == "This is a simple method."
            assert response.original_content == "This is a complex algorithm."