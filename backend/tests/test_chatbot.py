import pytest
import asyncio
from unittest.mock import AsyncMock, MagicMock, patch
from src.services.chatbot.rag_service import RAGService
from src.services.chatbot.vector_store_service import VectorStoreService
from src.services.chatbot.query_processor import QueryProcessor
from src.services.chatbot.content_chunker import ContentChunker


class TestRAGService:
    """Test cases for RAG chatbot service"""

    @pytest.fixture
    def rag_service(self):
        return RAGService()

    @pytest.mark.asyncio
    async def test_create_chatbot_session(self, rag_service):
        """Test creating a new chatbot session"""
        session = await rag_service.create_chatbot_session('textbook-123', 'user-456')

        assert session is not None
        assert session.textbook_id == 'textbook-123'
        assert session.user_id == 'user-456'
        assert session.is_active is True

    @pytest.mark.asyncio
    async def test_process_query(self, rag_service):
        """Test processing a user query with RAG"""
        # Mock session
        mock_session = MagicMock()
        mock_session.textbook_id = 'textbook-123'

        with patch.object(rag_service, '_retrieve_context', return_value=[
            {
                'content': 'Physical AI content about ROS 2',
                'source': 'textbook',
                'page': 1,
                'section': 'Introduction'
            }
        ]):
            with patch.object(rag_service, '_generate_response', return_value='This is a response based on the textbook content'):
                response = await rag_service.process_query(mock_session, 'What is ROS 2?')

                assert response is not None
                assert 'response' in response or isinstance(response, str)

    @pytest.mark.asyncio
    async def test_index_textbook_content(self, rag_service):
        """Test indexing textbook content for RAG search"""
        from src.models.textbook import Textbook
        from src.models.chapter import Chapter
        from src.models.section import Section

        mock_textbook = Textbook(
            id='textbook-123',
            title='Physical AI Textbook',
            subject='Physical AI',
            difficulty='intermediate',
            chapters=[
                Chapter(
                    id='chapter-1',
                    title='Introduction to ROS 2',
                    sections=[
                        Section(
                            id='section-1',
                            title='ROS 2 Basics',
                            content='ROS 2 is a robotics middleware that provides services like hardware abstraction, device drivers, libraries, visualizers, message-passing, package management, and more.'
                        )
                    ]
                )
            ]
        )

        result = await rag_service.index_textbook_content(mock_textbook)

        assert result is True
        assert mock_textbook.rag_indexed is True

    @pytest.mark.asyncio
    async def test_search_context(self, rag_service):
        """Test searching for relevant context in indexed content"""
        # This test would require pre-indexed content
        # For now, we'll test the method exists and returns expected format
        result = await rag_service.search_context('textbook-123', 'ROS 2', top_k=3)

        assert isinstance(result, list)


class TestVectorStoreService:
    """Test cases for vector store service"""

    @pytest.fixture
    def vector_store_service(self):
        return VectorStoreService()

    @pytest.mark.asyncio
    async def test_add_content_chunks(self, vector_store_service):
        """Test adding content chunks to vector store"""
        chunks = [
            {
                'id': 'chunk-1',
                'content': 'Physical AI content about ROS 2',
                'source': 'textbook-123/chapter-1',
                'metadata': {
                    'chapter_title': 'Introduction to ROS 2',
                    'section_title': 'ROS 2 Basics'
                }
            }
        ]

        result = await vector_store_service.add_content_chunks('textbook-123', chunks)

        assert result is True

    @pytest.mark.asyncio
    async def test_search_similar_chunks(self, vector_store_service):
        """Test searching for similar content chunks"""
        # Add some content first
        chunks = [
            {
                'id': 'chunk-1',
                'content': 'Physical AI content about ROS 2',
                'source': 'textbook-123/chapter-1',
                'metadata': {
                    'chapter_title': 'Introduction to ROS 2',
                    'section_title': 'ROS 2 Basics'
                }
            }
        ]
        await vector_store_service.add_content_chunks('textbook-123', chunks)

        # Now search
        results = await vector_store_service.search_similar_chunks('textbook-123', 'ROS 2', top_k=1)

        assert isinstance(results, list)
        assert len(results) <= 1


class TestQueryProcessor:
    """Test cases for query processing service"""

    @pytest.fixture
    def query_processor(self):
        return QueryProcessor()

    @pytest.mark.asyncio
    async def test_process_query(self, query_processor):
        """Test processing a user query"""
        result = await query_processor.process_query('What is ROS 2?')

        assert result is not None
        assert 'original_query' in result
        assert 'processed_query' in result
        assert 'query_type' in result
        assert 'intent' in result

    @pytest.mark.asyncio
    async def test_analyze_query(self, query_processor):
        """Test query analysis"""
        result = await query_processor._analyze_query('Explain how ROS 2 works')

        assert result is not None
        assert 'sentiment' in result
        assert 'entities' in result
        assert 'keywords' in result

    @pytest.mark.asyncio
    async def test_determine_query_type(self, query_processor):
        """Test determining query type"""
        factual_result = await query_processor._determine_query_type('What is ROS 2?')
        conceptual_result = await query_processor._determine_query_type('Why is ROS 2 important?')

        assert factual_result in ['factual', 'general']
        assert conceptual_result in ['conceptual', 'general']

    @pytest.mark.asyncio
    async def test_extract_entities(self, query_processor):
        """Test entity extraction from query"""
        entities = await query_processor.extract_entities('Explain ROS 2 and Gazebo simulation')

        assert isinstance(entities, list)


class TestContentChunker:
    """Test cases for content chunking service"""

    @pytest.fixture
    def content_chunker(self):
        return ContentChunker(max_chunk_size=100, overlap_size=10)

    def test_chunk_text_simple(self, content_chunker):
        """Test chunking simple text"""
        text = "This is a sample text for chunking. It contains multiple sentences. We will test how it gets divided."
        chunks = content_chunker._chunk_text(text, 'test-base', 'Chapter 1', 'Section 1')

        assert len(chunks) > 0
        assert all(len(chunk.content.split()) <= 100 for chunk in chunks)

    def test_chunk_chapter(self, content_chunker):
        """Test chunking a chapter"""
        chapter = {
            'id': 'ch-1',
            'title': 'Introduction to Physical AI',
            'content': 'Physical AI is the intersection of artificial intelligence and robotics. It involves creating intelligent systems that can interact with the physical world. This field combines machine learning, computer vision, and robotics to create embodied intelligence.',
            'sections': [
                {
                    'id': 'sec-1',
                    'title': 'What is Physical AI?',
                    'content': 'Physical AI refers to AI systems that function in the physical world and comprehend physical laws.'
                }
            ]
        }

        chunks = content_chunker.chunk_chapter(chapter, 'textbook-123')

        assert len(chunks) > 0
        assert all(chunk.chapter == 'Introduction to Physical AI' for chunk in chunks)

    def test_chunk_textbook_content(self, content_chunker):
        """Test chunking entire textbook content"""
        chapters = [
            {
                'id': 'ch-1',
                'title': 'Introduction to Physical AI',
                'content': 'Physical AI is the intersection of artificial intelligence and robotics.',
                'sections': [
                    {
                        'id': 'sec-1',
                        'title': 'What is Physical AI?',
                        'content': 'Physical AI refers to AI systems that function in the physical world.'
                    }
                ]
            }
        ]

        chunks = content_chunker.chunk_textbook_content('textbook-123', chapters)

        assert len(chunks) > 0