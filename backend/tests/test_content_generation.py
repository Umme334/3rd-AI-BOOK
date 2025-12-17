import pytest
import asyncio
from unittest.mock import AsyncMock, MagicMock, patch
from src.services.content_generation.generation_service import ContentGenerationService
from src.services.content_generation.physical_ai_content_service import PhysicalAIContentService
from src.services.content_generation.difficulty_service import DifficultyService
from src.services.content_generation.interactive_service import InteractiveService


class TestContentGenerationService:
    """Test cases for content generation service"""

    @pytest.fixture
    def content_service(self):
        return ContentGenerationService()

    @pytest.mark.asyncio
    async def test_generate_textbook_content(self, content_service):
        """Test generating textbook content with parameters"""
        params = {
            'subject': 'Physical AI',
            'difficulty': 'intermediate',
            'target_audience': 'students',
            'length': 5,
            'additional_topics': 'ROS 2, Gazebo'
        }

        result = await content_service.generate_textbook_content(params)

        assert result is not None
        assert 'chapters' in result
        assert len(result['chapters']) == params['length']

    @pytest.mark.asyncio
    async def test_generate_chapter_content(self, content_service):
        """Test generating individual chapter content"""
        chapter_params = {
            'title': 'Introduction to ROS 2',
            'difficulty': 'beginner',
            'module_type': 'ROS 2'
        }

        result = await content_service.generate_chapter_content(chapter_params)

        assert result is not None
        assert 'title' in result
        assert 'sections' in result
        assert result['title'] == chapter_params['title']

    @pytest.mark.asyncio
    async def test_generate_section_content(self, content_service):
        """Test generating individual section content"""
        section_params = {
            'title': 'ROS 2 Nodes',
            'difficulty': 'intermediate',
            'content_type': 'text'
        }

        result = await content_service.generate_section_content(section_params)

        assert result is not None
        assert 'content' in result
        assert len(result['content']) > 0


class TestPhysicalAIContentService:
    """Test cases for Physical AI specific content generation"""

    @pytest.fixture
    def physical_ai_service(self):
        return PhysicalAIContentService()

    @pytest.mark.asyncio
    async def test_generate_ros2_content(self, physical_ai_service):
        """Test generating ROS 2 specific content"""
        content = await physical_ai_service.generate_ros2_content('beginner')

        assert content is not None
        assert 'ROS 2' in content

    @pytest.mark.asyncio
    async def test_generate_gazebo_content(self, physical_ai_service):
        """Test generating Gazebo specific content"""
        content = await physical_ai_service.generate_gazebo_content('intermediate')

        assert content is not None
        assert 'Gazebo' in content

    @pytest.mark.asyncio
    async def test_generate_nvidia_isaac_content(self, physical_ai_service):
        """Test generating NVIDIA Isaac specific content"""
        content = await physical_ai_service.generate_nvidia_isaac_content('advanced')

        assert content is not None
        assert 'NVIDIA Isaac' in content

    @pytest.mark.asyncio
    async def test_generate_vla_content(self, physical_ai_service):
        """Test generating Vision-Language-Action content"""
        content = await physical_ai_service.generate_vla_content('intermediate')

        assert content is not None
        assert 'Vision-Language-Action' in content


class TestDifficultyService:
    """Test cases for difficulty-based content customization"""

    @pytest.fixture
    def difficulty_service(self):
        return DifficultyService()

    def test_adjust_content_for_beginner(self, difficulty_service):
        """Test content adjustment for beginner level"""
        original_content = "Complex algorithm implementation using advanced mathematical concepts"

        adjusted_content = difficulty_service.adjust_content_for_difficulty(
            original_content, 'beginner'
        )

        assert 'Complex' not in adjusted_content or 'simplified' in adjusted_content.lower()

    def test_adjust_content_for_advanced(self, difficulty_service):
        """Test content adjustment for advanced level"""
        original_content = "Basic concept explanation"

        adjusted_content = difficulty_service.adjust_content_for_difficulty(
            original_content, 'advanced'
        )

        assert 'advanced' in adjusted_content.lower() or 'complex' in adjusted_content.lower()

    def test_get_difficulty_prompts(self, difficulty_service):
        """Test getting difficulty-specific prompts"""
        prompts = difficulty_service.get_difficulty_prompts()

        assert 'beginner' in prompts
        assert 'intermediate' in prompts
        assert 'advanced' in prompts


class TestInteractiveService:
    """Test cases for interactive element generation"""

    @pytest.fixture
    def interactive_service(self):
        return InteractiveService()

    @pytest.mark.asyncio
    async def test_generate_quiz(self, interactive_service):
        """Test generating quiz interactive elements"""
        quiz_params = {
            'topic': 'ROS 2',
            'difficulty': 'intermediate',
            'question_count': 5
        }

        quiz = await interactive_service.generate_quiz(quiz_params)

        assert quiz is not None
        assert 'type' in quiz
        assert quiz['type'] == 'quiz'
        assert 'questions' in quiz

    @pytest.mark.asyncio
    async def test_generate_summary(self, interactive_service):
        """Test generating summary interactive elements"""
        content = "This is a chapter about Physical AI concepts"

        summary = await interactive_service.generate_summary(content)

        assert summary is not None
        assert 'type' in summary
        assert summary['type'] == 'summary'
        assert 'content' in summary

    @pytest.mark.asyncio
    async def test_generate_learning_booster(self, interactive_service):
        """Test generating learning booster elements"""
        topic = 'Computer Vision'

        booster = await interactive_service.generate_learning_booster(topic)

        assert booster is not None
        assert 'type' in booster
        assert booster['type'] == 'booster'