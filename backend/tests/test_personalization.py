import pytest
import asyncio
from unittest.mock import AsyncMock, MagicMock, patch
from src.services.personalization.personalization_service import PersonalizationService
from src.services.personalization.content_adapter import ContentAdapter


class TestPersonalizationService:
    """Test cases for personalization service"""

    @pytest.fixture
    def personalization_service(self):
        return PersonalizationService()

    @pytest.mark.asyncio
    async def test_get_personalized_content(self, personalization_service):
        """Test getting personalized content based on user profile"""
        from src.schemas.personalization_schemas import PersonalizationRequest

        request = PersonalizationRequest(
            user_id='user-123',
            textbook_id='textbook-456',
            chapter_id='chapter-789',
            personalization_level='adaptive',
            learning_style='visual',
            difficulty_override='intermediate'
        )

        # Mock the internal methods
        with patch.object(personalization_service, '_get_user_profile') as mock_get_user:
            mock_user_profile = MagicMock()
            mock_user_profile.software_experience = 'intermediate'
            mock_user_profile.hardware_experience = 'beginner'
            mock_user_profile.programming_languages = ['Python', 'C++']
            mock_user_profile.hardware_platforms = ['NVIDIA Jetson', 'Raspberry Pi']
            mock_get_user.return_value = mock_user_profile

            with patch.object(personalization_service, '_get_content', return_value='Original content about ROS 2'):
                with patch.object(personalization_service.content_adapter, 'adapt_content', return_value='Adapted content for intermediate user'):
                    result = await personalization_service.get_personalized_content(request)

                    assert result is not None
                    assert 'personalized_content' in result
                    assert 'original_content' in result

    @pytest.mark.asyncio
    async def test_create_personalization_profile(self, personalization_service):
        """Test creating a personalization profile"""
        profile = await personalization_service.create_personalization_profile('user-123', 'textbook-456')

        assert profile is not None
        assert profile.user_id == 'user-123'
        assert 'user_background' in profile
        assert 'learning_preferences' in profile
        assert 'content_adaptation_rules' in profile

    @pytest.mark.asyncio
    async def test_adapt_content(self, personalization_service):
        """Test adapting specific content"""
        from src.schemas.personalization_schemas import ContentAdaptationRequest

        request = ContentAdaptationRequest(
            content='This is original content about Physical AI',
            user_profile={
                'software_experience': 'beginner',
                'hardware_experience': 'beginner',
                'programming_languages': ['Python']
            },
            context={
                'difficulty': 'beginner',
                'example_preference': 'code_implementation'
            }
        )

        result = await personalization_service.get_content_adaptation(request)

        assert result is not None
        assert result.original_content == request.content
        assert 'adapted_content' in result

    @pytest.mark.asyncio
    async def test_personalize_textbook(self, personalization_service):
        """Test personalizing an entire textbook"""
        from src.models.textbook import Textbook
        from src.models.chapter import Chapter
        from src.models.user_profile import UserProfile

        mock_textbook = Textbook(
            id='textbook-123',
            title='Physical AI Textbook',
            subject='Physical AI',
            difficulty='intermediate',
            chapters=[
                Chapter(
                    id='chapter-1',
                    title='Introduction to ROS 2',
                    content='Original content about ROS 2'
                )
            ]
        )

        mock_user_profile = UserProfile(
            id='user-123',
            email='user@example.com',
            software_experience='intermediate',
            hardware_experience='beginner',
            programming_languages=['Python', 'C++'],
            hardware_platforms=['NVIDIA Jetson', 'Raspberry Pi'],
            robotics_experience='basic',
            math_background='intermediate',
            primary_goal='Learn humanoid robotics'
        )

        result = await personalization_service.personalize_textbook(mock_textbook, mock_user_profile)

        assert result is not None
        assert result.id == mock_textbook.id
        assert hasattr(result, 'personalization_enabled')

    @pytest.mark.asyncio
    async def test_personalize_chapter(self, personalization_service):
        """Test personalizing a single chapter"""
        from src.models.chapter import Chapter
        from src.models.user_profile import UserProfile

        mock_chapter = Chapter(
            id='chapter-1',
            title='Introduction to ROS 2',
            content='Original content about ROS 2'
        )

        mock_user_profile = UserProfile(
            id='user-123',
            email='user@example.com',
            software_experience='beginner',
            hardware_experience='beginner',
            programming_languages=['Python'],
            hardware_platforms=['Raspberry Pi'],
            robotics_experience='none',
            math_background='basic',
            primary_goal='Learn basics'
        )

        result = await personalization_service.personalize_chapter(mock_chapter, mock_user_profile)

        assert result is not None
        assert 'title' in result
        assert 'sections' in result


class TestContentAdapter:
    """Test cases for content adaptation service"""

    @pytest.fixture
    def content_adapter(self):
        return ContentAdapter()

    @pytest.mark.asyncio
    async def test_adapt_content(self, content_adapter):
        """Test adapting content based on user profile"""
        content = "This is complex content about ROS 2 nodes and topics"
        user_profile = {
            'software_experience': 'beginner',
            'hardware_experience': 'beginner',
            'programming_languages': ['Python'],
            'robotics_experience': 'none'
        }
        context = {
            'difficulty': 'beginner',
            'content_type': 'chapter',
            'subject': 'ROS 2'
        }

        adapted_content = await content_adapter.adapt_content(content, user_profile, context)

        assert adapted_content is not None
        assert isinstance(adapted_content, str)
        assert len(adapted_content) > 0

    @pytest.mark.asyncio
    async def test_adapt_content_for_advanced_user(self, content_adapter):
        """Test adapting content for advanced user"""
        content = "Basic introduction to ROS 2"
        user_profile = {
            'software_experience': 'advanced',
            'hardware_experience': 'advanced',
            'programming_languages': ['Python', 'C++', 'ROS'],
            'robotics_experience': 'advanced'
        }
        context = {
            'difficulty': 'advanced',
            'content_type': 'chapter',
            'subject': 'ROS 2'
        }

        adapted_content = await content_adapter.adapt_content(content, user_profile, context)

        assert adapted_content is not None
        # Advanced adaptation might add more technical depth
        assert isinstance(adapted_content, str)

    @pytest.mark.asyncio
    async def test_adapt_content_with_math_background(self, content_adapter):
        """Test adapting content based on math background"""
        content = "Conceptual explanation of robot kinematics"
        user_profile = {
            'software_experience': 'intermediate',
            'hardware_experience': 'intermediate',
            'math_background': 'advanced',
            'robotics_experience': 'intermediate'
        }
        context = {
            'difficulty': 'intermediate',
            'content_type': 'section',
            'subject': 'Robot Kinematics'
        }

        adapted_content = await content_adapter.adapt_content(content, user_profile, context)

        assert adapted_content is not None
        assert isinstance(adapted_content, str)

    def test_determine_adaptation_strategy(self, content_adapter):
        """Test determining adaptation strategy based on user profile"""
        user_profile = {
            'software_experience': 'beginner',
            'hardware_experience': 'beginner',
            'programming_languages': ['Python']
        }

        strategy = content_adapter._determine_adaptation_strategy(user_profile)

        assert strategy is not None
        assert 'difficulty_adjustment' in strategy
        assert 'content_pacing' in strategy

    def test_apply_difficulty_adjustment(self, content_adapter):
        """Test applying difficulty adjustment to content"""
        original_content = "Complex algorithm implementation"
        user_profile = {
            'software_experience': 'beginner',
            'robotics_experience': 'none'
        }

        adjusted_content = content_adapter._apply_difficulty_adjustment(
            original_content,
            user_profile,
            'beginner'
        )

        assert adjusted_content is not None
        assert isinstance(adjusted_content, str)

    def test_apply_example_preference(self, content_adapter):
        """Test applying example preference to content"""
        content = "Theoretical explanation of concepts"
        user_profile = {
            'programming_languages': ['Python', 'C++'],
            'hardware_platforms': ['NVIDIA Jetson']
        }

        content_with_examples = content_adapter._apply_example_preference(
            content,
            user_profile
        )

        assert content_with_examples is not None
        assert isinstance(content_with_examples, str)

    def test_calculate_user_comprehension_level(self, content_adapter):
        """Test calculating user comprehension level"""
        user_profile = {
            'software_experience': 'intermediate',
            'hardware_experience': 'beginner',
            'math_background': 'advanced',
            'robotics_experience': 'intermediate'
        }

        comprehension_level = content_adapter._calculate_user_comprehension_level(user_profile)

        assert comprehension_level is not None
        assert isinstance(comprehension_level, float)
        assert 0.0 <= comprehension_level <= 1.0