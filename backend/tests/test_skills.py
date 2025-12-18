import pytest
import asyncio
from unittest.mock import AsyncMock, MagicMock, patch
from src.skills.skills_manager import SkillsManager
from src.skills.base_skill import BaseSkill, SkillResult
from src.skills.retrieval_skill import TextRetrievalSkill
from src.skills.content_skill import ContentIndexingSkill
from src.skills.generation_skill import TextGenerationSkill


class TestSkillsManager:
    """Test cases for Skills Manager"""

    @pytest.fixture
    def skills_manager(self):
        return SkillsManager()

    def test_initialize_default_skills(self, skills_manager):
        """Test initializing default skills"""
        # Skills should be automatically initialized
        skills = skills_manager.get_available_skills()
        assert len(skills) > 0
        assert any(skill['skill_id'] == 'text_retrieval' for skill in skills)
        assert any(skill['skill_id'] == 'content_indexing' for skill in skills)
        assert any(skill['skill_id'] == 'text_generation' for skill in skills)

    @pytest.mark.asyncio
    async def test_execute_skill(self, skills_manager):
        """Test executing a skill by ID"""
        # Mock the skill execution
        with patch.object(skills_manager, 'get_skill') as mock_get_skill:
            mock_skill = MagicMock()
            mock_skill.validate_inputs = AsyncMock(return_value=True)
            mock_skill.execute = AsyncMock(return_value=SkillResult(
                success=True,
                data="test result",
                message="Skill executed successfully"
            ))
            mock_get_skill.return_value = mock_skill

            result = await skills_manager.execute_skill('text_retrieval', query="test query")

            assert result is not None
            assert result.success is True
            assert result.data == "test result"

    def test_get_skill(self, skills_manager):
        """Test getting a specific skill by ID"""
        skill = skills_manager.get_skill('text_retrieval')
        assert skill is not None
        assert hasattr(skill, 'skill_id')
        assert skill.skill_id == 'text_retrieval'

    @pytest.mark.asyncio
    async def test_execute_skill_chain(self, skills_manager):
        """Test executing skills in a chain"""
        skill_chain = [
            {
                "skill_id": "text_retrieval",
                "kwargs": {"query": "test query", "textbook_id": "test-book", "top_k": 3}
            },
            {
                "skill_id": "text_generation",
                "kwargs": {"query": "test query", "context": "test context"}
            }
        ]

        # Mock the execution of each skill in the chain
        with patch.object(skills_manager, 'execute_skill') as mock_execute:
            mock_execute.return_value = SkillResult(
                success=True,
                data="test result",
                message="Skill executed successfully"
            )

            results = await skills_manager.execute_skill_chain(skill_chain)

            assert len(results) == 2
            assert all(result.success for result in results)


class TestBaseSkill:
    """Test cases for Base Skill"""

    @pytest.fixture
    def base_skill(self):
        class TestSkill(BaseSkill):
            def __init__(self):
                super().__init__(
                    skill_id="test_skill",
                    name="Test Skill",
                    description="A test skill for testing purposes",
                    version="1.0.0"
                )

            async def execute(self, **kwargs):
                return SkillResult(
                    success=True,
                    data=kwargs.get("test_data", "default result"),
                    message="Test skill executed successfully"
                )

        return TestSkill()

    @pytest.mark.asyncio
    async def test_skill_execution(self, base_skill):
        """Test executing the base skill"""
        result = await base_skill.execute(test_data="test value")

        assert result.success is True
        assert result.data == "test value"
        assert "executed successfully" in result.message

    @pytest.mark.asyncio
    async def test_skill_validation(self, base_skill):
        """Test skill input validation"""
        is_valid = await base_skill.validate_inputs(test_param="test_value")
        assert is_valid is True  # Base skill validation returns True by default


class TestTextRetrievalSkill:
    """Test cases for Text Retrieval Skill"""

    @pytest.fixture
    def retrieval_skill(self):
        return TextRetrievalSkill()

    @pytest.mark.asyncio
    async def test_execute(self, retrieval_skill):
        """Test executing the text retrieval skill"""
        # Mock the RAG service's search_context method
        with patch.object(retrieval_skill.rag_service, 'search_context', return_value=[
            {
                "id": "chunk-1",
                "content": "Test content for retrieval",
                "source": "test-source",
                "chapter": "Test Chapter",
                "section": "Test Section",
                "similarity_score": 0.9
            }
        ]):
            result = await retrieval_skill.execute(
                query="test query",
                textbook_id="test-book",
                top_k=5
            )

            assert result.success is True
            assert result.data is not None
            assert len(result.data) == 1
            assert result.data[0]["content"] == "Test content for retrieval"

    @pytest.mark.asyncio
    async def test_validate_inputs(self, retrieval_skill):
        """Test input validation for text retrieval skill"""
        # Valid inputs
        valid = await retrieval_skill.validate_inputs(
            query="test query",
            textbook_id="test-book",
            top_k=5
        )
        assert valid is True

        # Invalid inputs
        invalid = await retrieval_skill.validate_inputs(
            query="",
            textbook_id="test-book"
        )
        assert invalid is False


class TestContentIndexingSkill:
    """Test cases for Content Indexing Skill"""

    @pytest.fixture
    def indexing_skill(self):
        return ContentIndexingSkill()

    @pytest.mark.asyncio
    async def test_execute(self, indexing_skill):
        """Test executing the content indexing skill"""
        # Mock the RAG service's index_textbook_content method
        mock_textbook = MagicMock()
        mock_textbook.id = "test-book"
        mock_textbook.title = "Test Textbook"

        with patch.object(indexing_skill.rag_service, 'index_textbook_content', return_value=True):
            result = await indexing_skill.execute(textbook=mock_textbook)

            assert result.success is True
            assert result.data is True

    @pytest.mark.asyncio
    async def test_validate_inputs(self, indexing_skill):
        """Test input validation for content indexing skill"""
        from src.models.textbook import Textbook

        mock_textbook = Textbook(
            id="test-book",
            title="Test Textbook",
            subject="Test Subject",
            difficulty="intermediate"
        )

        # Valid inputs
        valid = await indexing_skill.validate_inputs(textbook=mock_textbook)
        assert valid is True

        # Invalid inputs
        invalid = await indexing_skill.validate_inputs(textbook=None)
        assert invalid is False


class TestTextGenerationSkill:
    """Test cases for Text Generation Skill"""

    @pytest.fixture
    def generation_skill(self):
        return TextGenerationSkill()

    @pytest.mark.asyncio
    async def test_execute(self, generation_skill):
        """Test executing the text generation skill"""
        # Mock the RAG service's _generate_response method
        with patch.object(generation_skill.rag_service, '_generate_response', return_value="Generated response based on context"):
            result = await generation_skill.execute(
                query="test query",
                context="test context",
                selected_text="selected text"
            )

            assert result.success is True
            assert "Generated response" in result.data

    @pytest.mark.asyncio
    async def test_validate_inputs(self, generation_skill):
        """Test input validation for text generation skill"""
        # Valid inputs
        valid = await generation_skill.validate_inputs(
            query="test query",
            context="test context"
        )
        assert valid is True

        # Invalid inputs
        invalid = await generation_skill.validate_inputs(
            query="",
            context=""
        )
        assert invalid is False


class TestSkillResult:
    """Test cases for Skill Result"""

    def test_skill_result_creation(self):
        """Test creating a skill result"""
        result = SkillResult(
            success=True,
            data="test data",
            message="Test message"
        )

        assert result.success is True
        assert result.data == "test data"
        assert result.message == "Test message"

    def test_skill_result_defaults(self):
        """Test skill result with default values"""
        result = SkillResult(success=False)

        assert result.success is False
        assert result.data is None
        assert result.message == ""