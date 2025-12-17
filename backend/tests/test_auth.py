import pytest
import asyncio
from unittest.mock import AsyncMock, MagicMock, patch
from src.services.auth.user_service import UserService
from src.services.auth.background_capture_service import BackgroundCaptureService


class TestUserService:
    """Test cases for user authentication service"""

    @pytest.fixture
    def user_service(self):
        return UserService()

    @pytest.mark.asyncio
    async def test_create_user(self, user_service):
        """Test creating a new user"""
        user_data = {
            'email': 'test@example.com',
            'password': 'securepassword123',
            'first_name': 'John',
            'last_name': 'Doe',
            'software_experience': 'intermediate',
            'hardware_experience': 'beginner',
            'programming_languages': ['Python', 'C++'],
            'hardware_platforms': ['NVIDIA Jetson', 'Raspberry Pi'],
            'robotics_experience': 'basic',
            'math_background': 'intermediate',
            'primary_goal': 'Learn humanoid robotics'
        }

        result = user_service.create_user(user_data)

        assert result is not None
        assert 'user_id' in result
        assert result['email'] == user_data['email']

    @pytest.mark.asyncio
    async def test_authenticate_user(self, user_service):
        """Test user authentication"""
        credentials = {
            'email': 'test@example.com',
            'password': 'securepassword123'
        }

        # Mock successful authentication
        with patch.object(user_service, '_validate_credentials', return_value=True):
            with patch.object(user_service, '_get_user_by_email', return_value={
                'id': 'user-123',
                'email': 'test@example.com',
                'password_hash': 'hashed_password'
            }):
                result = user_service.authenticate_user(credentials)

                assert result is not None
                assert 'id' in result
                assert result['email'] == credentials['email']

    @pytest.mark.asyncio
    async def test_create_access_token(self, user_service):
        """Test creating access token for user"""
        user_id = 'user-123'

        token_data = user_service.create_access_token(user_id)

        assert token_data is not None
        assert 'access_token' in token_data
        assert 'token_type' in token_data
        assert token_data['token_type'] == 'Bearer'

    @pytest.mark.asyncio
    async def test_get_user_profile_response(self, user_service):
        """Test getting user profile response"""
        user_id = 'user-123'

        # Mock user profile data
        with patch.object(user_service, '_get_user_profile', return_value={
            'id': user_id,
            'email': 'test@example.com',
            'first_name': 'John',
            'last_name': 'Doe',
            'software_experience': 'intermediate',
            'hardware_experience': 'beginner'
        }):
            profile_response = await user_service.get_user_profile_response(user_id)

            assert profile_response is not None
            assert profile_response.user_id == user_id
            assert profile_response.email == 'test@example.com'

    @pytest.mark.asyncio
    async def test_update_user_profile(self, user_service):
        """Test updating user profile"""
        user_id = 'user-123'
        update_data = {
            'software_experience': 'advanced',
            'primary_goal': 'Research in humanoid robotics'
        }

        # Mock update operation
        with patch.object(user_service, '_update_user_in_db', return_value={
            'id': user_id,
            'email': 'test@example.com',
            'software_experience': 'advanced',
            'primary_goal': 'Research in humanoid robotics'
        }):
            profile_response = await user_service.update_user_profile(user_id, update_data)

            assert profile_response is not None
            assert profile_response.software_experience == 'advanced'


class TestBackgroundCaptureService:
    """Test cases for background capture service"""

    @pytest.fixture
    def background_service(self):
        return BackgroundCaptureService()

    def test_validate_background_info(self, background_service):
        """Test validating user background information"""
        background_data = {
            'software_experience': 'intermediate',
            'hardware_experience': 'beginner',
            'programming_languages': ['Python', 'C++'],
            'hardware_platforms': ['NVIDIA Jetson', 'Raspberry Pi'],
            'robotics_experience': 'basic',
            'math_background': 'intermediate',
            'primary_goal': 'Learn humanoid robotics'
        }

        validation_errors = background_service.validate_background_info(background_data)

        assert validation_errors is not None
        assert isinstance(validation_errors, dict)
        # Should have no errors for valid data
        assert not any(validation_errors.values())

    def test_validate_background_info_invalid(self, background_service):
        """Test validating invalid background information"""
        background_data = {
            'software_experience': 'invalid_level',
            'hardware_experience': 'invalid_level',
            'programming_languages': [],
            'robotics_experience': 'invalid_level',
            'math_background': 'invalid_level'
        }

        validation_errors = background_service.validate_background_info(background_data)

        assert validation_errors is not None
        # Should have errors for invalid data
        assert any(validation_errors.values())

    def test_categorize_background_info(self, background_service):
        """Test categorizing background information"""
        background_data = {
            'software_experience': 'intermediate',
            'hardware_experience': 'beginner',
            'programming_languages': ['Python', 'C++'],
            'hardware_platforms': ['NVIDIA Jetson', 'Raspberry Pi'],
            'robotics_experience': 'basic',
            'math_background': 'intermediate',
            'primary_goal': 'Learn humanoid robotics'
        }

        categorized = background_service.categorize_background_info(background_data)

        assert categorized is not None
        assert 'experience_level' in categorized
        assert 'technical_background' in categorized
        assert 'goals' in categorized

    def test_get_experience_level_score(self, background_service):
        """Test getting experience level score"""
        experience_levels = ['beginner', 'intermediate', 'advanced']

        for level in experience_levels:
            score = background_service.get_experience_level_score(level)
            assert 0 <= score <= 1

    def test_generate_learning_path(self, background_service):
        """Test generating learning path based on background"""
        user_background = {
            'software_experience': 'beginner',
            'hardware_experience': 'beginner',
            'robotics_experience': 'none',
            'math_background': 'basic',
            'primary_goal': 'Learn humanoid robotics'
        }

        learning_path = background_service.generate_learning_path(user_background)

        assert learning_path is not None
        assert 'recommended_path' in learning_path
        assert 'prerequisites' in learning_path
        assert 'timeline' in learning_path

    def test_validate_programming_languages(self, background_service):
        """Test validating programming languages"""
        valid_languages = ['Python', 'C++', 'ROS', 'MATLAB']
        invalid_languages = ['InvalidLang1', 'InvalidLang2']

        valid_result = background_service.validate_programming_languages(valid_languages)
        invalid_result = background_service.validate_programming_languages(invalid_languages)

        assert valid_result == []
        assert len(invalid_result) == len(invalid_languages)

    def test_validate_hardware_platforms(self, background_service):
        """Test validating hardware platforms"""
        valid_platforms = ['NVIDIA Jetson', 'Raspberry Pi', 'Arduino']
        invalid_platforms = ['InvalidPlatform1', 'InvalidPlatform2']

        valid_result = background_service.validate_hardware_platforms(valid_platforms)
        invalid_result = background_service.validate_hardware_platforms(invalid_platforms)

        assert valid_result == []
        assert len(invalid_result) == len(invalid_platforms)