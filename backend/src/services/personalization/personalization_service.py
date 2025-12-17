from typing import Dict, Any, List, Optional
from datetime import datetime
import logging

from ...models.textbook import Textbook
from ...models.chapter import Chapter
from ...models.section import Section
from ...models.user_profile import UserProfile
from ...services.openai_client import OpenAIClient
from ...schemas.personalization_schemas import (
    PersonalizationRequest,
    PersonalizationResponse,
    ContentAdaptationRequest,
    ContentAdaptationResponse,
    PersonalizationProfile
)
from ...services.auth.user_service import UserService
from ...services.auth.background_capture_service import BackgroundCaptureService
from ..base_service import BaseService


logger = logging.getLogger(__name__)


class PersonalizationService(BaseService):
    """
    Service for content personalization based on user background.
    """

    def __init__(self):
        super().__init__()
        self.openai_client = OpenAIClient()
        self.user_service = UserService()
        self.background_service = BackgroundCaptureService()
        self.personalization_profiles = {}  # In-memory cache of personalization profiles

    async def get_personalized_content(
        self,
        request: PersonalizationRequest
    ) -> PersonalizationResponse:
        """
        Get personalized content based on user profile and preferences
        """
        try:
            # Get user profile
            user_profile_response = await self.user_service.get_user_profile_response(request.user_id)
            if not user_profile_response:
                raise ValueError(f"User profile not found for user_id: {request.user_id}")

            # Create personalization profile
            personalization_profile = await self._create_personalization_profile(
                request.user_id,
                user_profile_response.dict() if hasattr(user_profile_response, 'dict') else user_profile_response
            )

            # For now, we'll adapt content based on profile - in a real implementation
            # we would fetch the actual textbook/chapter content
            original_content = f"Original content for textbook {request.textbook_id}"
            if request.chapter_id:
                original_content += f", chapter {request.chapter_id}"
            if request.section_id:
                original_content += f", section {request.section_id}"

            # Adapt the content based on user profile
            adapted_content = await self._adapt_content(
                original_content,
                personalization_profile,
                request
            )

            # Calculate confidence score based on profile completeness
            confidence_score = self._calculate_confidence_score(personalization_profile)

            response = PersonalizationResponse(
                original_content=original_content,
                personalized_content=adapted_content,
                personalization_applied=True,
                adaptation_reasons=self._get_adaptation_reasons(personalization_profile),
                confidence_score=confidence_score,
                processing_time_ms=100.0,  # Placeholder
                user_profile_match=self._calculate_profile_match(personalization_profile)
            )

            return response

        except Exception as e:
            logger.error(f"Error in get_personalized_content: {str(e)}")
            raise

    async def _create_personalization_profile(
        self,
        user_id: str,
        user_profile_data: Dict[str, Any]
    ) -> PersonalizationProfile:
        """
        Create a personalization profile from user profile data
        """
        # Check if profile is already cached
        if user_id in self.personalization_profiles:
            return self.personalization_profiles[user_id]

        # Extract user background from profile
        user_background = {
            "software_experience": user_profile_data.get('software_experience'),
            "hardware_experience": user_profile_data.get('hardware_experience'),
            "programming_languages": user_profile_data.get('programming_languages', []),
            "hardware_platforms": user_profile_data.get('hardware_platforms', []),
            "robotics_experience": user_profile_data.get('robotics_experience'),
            "math_background": user_profile_data.get('math_background'),
            "primary_goal": user_profile_data.get('primary_goal'),
            "background_questions": user_profile_data.get('background_questions', {}),
            "profile_completed": user_profile_data.get('profile_completed', False)
        }

        # Create content adaptation rules based on user background
        adaptation_rules = self._create_adaptation_rules(user_background)

        profile = PersonalizationProfile(
            user_id=user_id,
            user_background=user_background,
            learning_preferences={},  # Would come from separate preferences storage
            content_adaptation_rules=adaptation_rules,
            last_calculated=datetime.now()
        )

        # Cache the profile
        self.personalization_profiles[user_id] = profile

        return profile

    def _create_adaptation_rules(self, user_background: Dict[str, Any]) -> Dict[str, Any]:
        """
        Create content adaptation rules based on user background
        """
        rules = {
            "difficulty_adaptation": self._determine_difficulty_adaptation(user_background),
            "example_preference": self._determine_example_preference(user_background),
            "explanation_style": self._determine_explanation_style(user_background),
            "content_pacing": self._determine_content_pacing(user_background),
            "prerequisite_checking": self._determine_prereq_checking(user_background)
        }

        return rules

    def _determine_difficulty_adaptation(self, user_background: Dict[str, Any]) -> str:
        """
        Determine difficulty adaptation level based on user background
        """
        exp_levels = [
            user_background.get('software_experience'),
            user_background.get('hardware_experience'),
            user_background.get('robotics_experience')
        ]

        # Map experience levels to numeric values
        exp_values = {
            'beginner': 1,
            'basic': 1,
            'intermediate': 2,
            'advanced': 3,
            'none': 0
        }

        # Calculate average experience level
        valid_exp = [exp_values.get(exp, 0) for exp in exp_levels if exp]
        if not valid_exp:
            return 'beginner'

        avg_exp = sum(valid_exp) / len(valid_exp)

        if avg_exp >= 2.5:
            return 'advanced'
        elif avg_exp >= 1.5:
            return 'intermediate'
        else:
            return 'beginner'

    def _determine_example_preference(self, user_background: Dict[str, Any]) -> str:
        """
        Determine preferred example types based on user background
        """
        programming_langs = user_background.get('programming_languages', [])
        hardware_platforms = user_background.get('hardware_platforms', [])

        # Check for specific preferences
        if 'Python' in programming_langs or 'ROS' in programming_langs:
            return 'code_implementation'
        elif 'MATLAB' in programming_langs:
            return 'mathematical_modeling'
        elif any(platform in hardware_platforms for platform in ['ROSbot', 'TurtleBot', 'NVIDIA Isaac']):
            return 'practical_implementation'
        else:
            return 'conceptual_explanation'

    def _determine_explanation_style(self, user_background: Dict[str, Any]) -> str:
        """
        Determine explanation style based on user background
        """
        math_bg = user_background.get('math_background', 'basic')
        robotics_exp = user_background.get('robotics_experience', 'none')

        if math_bg == 'advanced' and robotics_exp in ['intermediate', 'advanced']:
            return 'mathematical_formal'
        elif math_bg == 'basic' or robotics_exp == 'none':
            return 'intuitive_visual'
        else:
            return 'balanced_approach'

    def _determine_content_pacing(self, user_background: Dict[str, Any]) -> str:
        """
        Determine content pacing based on user background
        """
        primary_goal = user_background.get('primary_goal', 'education')

        if primary_goal in ['career', 'research']:
            return 'accelerated'
        elif primary_goal == 'hobby':
            return 'self_paced'
        else:
            return 'standard'

    def _determine_prereq_checking(self, user_background: Dict[str, Any]) -> bool:
        """
        Determine if prerequisite checking is needed based on user background
        """
        software_exp = user_background.get('software_experience', 'beginner')
        return software_exp == 'beginner'

    async def _adapt_content(
        self,
        original_content: str,
        profile: PersonalizationProfile,
        request: PersonalizationRequest
    ) -> str:
        """
        Adapt content based on personalization profile and request parameters
        """
        # Apply adaptations based on profile
        adapted_content = original_content

        # Apply difficulty adaptation
        difficulty_adaptation = profile.content_adaptation_rules.get('difficulty_adaptation')
        if request.difficulty_override:
            difficulty_adaptation = request.difficulty_override
        if difficulty_adaptation:
            if difficulty_adaptation == 'beginner':
                adapted_content = self._simplify_content(adapted_content)
            elif difficulty_adaptation == 'advanced':
                adapted_content = self._enrich_content(adapted_content)

        # Apply example preference
        example_pref = profile.content_adaptation_rules.get('example_preference')
        if example_pref:
            adapted_content = self._apply_example_preference(adapted_content, example_pref)

        # Apply explanation style
        explanation_style = profile.content_adaptation_rules.get('explanation_style')
        if explanation_style:
            adapted_content = self._apply_explanation_style(adapted_content, explanation_style)

        return adapted_content

    def _simplify_content(self, content: str) -> str:
        """
        Simplify content for beginner users
        """
        # Add more explanations and simpler language
        simplified = content.replace("advanced concept", "basic concept")
        simplified = simplified.replace("complex algorithm", "simple method")
        simplified = simplified.replace("mathematical proof", "intuitive explanation")

        # Add more examples and step-by-step breakdowns
        if "algorithm" in simplified.lower():
            simplified += "\n\nStep-by-step breakdown:\n1. First step\n2. Second step\n3. Final step"

        return simplified

    def _enrich_content(self, content: str) -> str:
        """
        Enrich content for advanced users
        """
        # Add more depth and complexity
        enriched = content.replace("basic concept", "advanced concept")
        enriched = enriched.replace("simple method", "complex algorithm")
        enriched = enriched.replace("intuitive explanation", "mathematical proof")

        # Add more technical details
        if "algorithm" in enriched.lower():
            enriched += "\n\nImplementation considerations:\n- Time complexity: O(n^2)\n- Space complexity: O(n)\n- Optimizations possible"

        return enriched

    def _apply_example_preference(self, content: str, preference: str) -> str:
        """
        Apply example preference to content
        """
        if preference == 'code_implementation':
            return content + "\n\nExample implementation:\n```python\n# Code example here\n```"
        elif preference == 'mathematical_modeling':
            return content + "\n\nMathematical representation:\nLet x be the variable..."
        elif preference == 'practical_implementation':
            return content + "\n\nHardware implementation:\nConnect pin 1 to pin 2..."
        else:  # conceptual_explanation
            return content + "\n\nConcept explained: This works because..."

    def _apply_explanation_style(self, content: str, style: str) -> str:
        """
        Apply explanation style to content
        """
        if style == 'mathematical_formal':
            return content + "\n\nFormal definition: Let S be a set..."
        elif style == 'intuitive_visual':
            return content + "\n\nThink of it this way: Imagine..."
        else:  # balanced_approach
            return content + "\n\nExplanation: Both conceptually and formally..."

    def _calculate_confidence_score(self, profile: PersonalizationProfile) -> float:
        """
        Calculate confidence in personalization based on profile completeness
        """
        background = profile.user_background
        completeness_score = 0
        total_fields = 0

        # Check various background fields for completeness
        fields_to_check = [
            'software_experience',
            'hardware_experience',
            'robotics_experience',
            'math_background',
            'primary_goal'
        ]

        for field in fields_to_check:
            if background.get(field):
                completeness_score += 1
            total_fields += 1

        # Check list fields
        list_fields = ['programming_languages', 'hardware_platforms']
        for field in list_fields:
            if background.get(field) and len(background[field]) > 0:
                completeness_score += 1
            total_fields += 1

        if total_fields == 0:
            return 0.3  # Default low confidence if no fields

        return min(0.95, completeness_score / total_fields)  # Cap at 0.95

    def _get_adaptation_reasons(self, profile: PersonalizationProfile) -> List[str]:
        """
        Get reasons for adaptations made to content
        """
        reasons = []

        difficulty = profile.content_adaptation_rules.get('difficulty_adaptation')
        if difficulty:
            reasons.append(f"Content adapted for {difficulty} level learners")

        example_pref = profile.content_adaptation_rules.get('example_preference')
        if example_pref:
            reasons.append(f"Examples tailored to {example_pref} preference")

        return reasons if reasons else ["Content adapted based on user profile"]

    def _calculate_profile_match(self, profile: PersonalizationProfile) -> float:
        """
        Calculate how well content matches user profile
        """
        # This is a simplified calculation
        # In a real implementation, this would compare content tags with user profile
        return profile.content_adaptation_rules.get('difficulty_adaptation') != 'beginner' and 0.8 or 0.6

    async def update_personalization_profile(self, user_id: str) -> PersonalizationProfile:
        """
        Update a personalization profile based on new user data
        """
        # Clear cached profile to force recalculation
        if user_id in self.personalization_profiles:
            del self.personalization_profiles[user_id]

        # Get updated user profile
        user_profile_response = await self.user_service.get_user_profile_response(user_id)
        if not user_profile_response:
            raise ValueError(f"User profile not found for user_id: {user_id}")

        # Recreate profile
        return await self._create_personalization_profile(
            user_id,
            user_profile_response.dict() if hasattr(user_profile_response, 'dict') else user_profile_response
        )

    async def get_content_adaptation(
        self,
        adaptation_request: ContentAdaptationRequest
    ) -> ContentAdaptationResponse:
        """
        Adapt specific content based on user profile and context
        """
        # Apply adaptation based on context
        adapted_content = adaptation_request.content

        # Apply difficulty adaptation if specified in context
        context_difficulty = adaptation_request.context.get('difficulty')
        if context_difficulty:
            if context_difficulty == 'beginner':
                adapted_content = self._simplify_content(adapted_content)
            elif context_difficulty == 'advanced':
                adapted_content = self._enrich_content(adapted_content)

        # Apply example preference from user profile
        user_example_pref = adaptation_request.context.get('example_preference')
        if user_example_pref:
            adapted_content = self._apply_example_preference(adapted_content, user_example_pref)

        return ContentAdaptationResponse(
            original_content=adaptation_request.content,
            adapted_content=adapted_content,
            adaptation_type="difficulty_and_examples",
            adaptation_reason="Content adapted based on user profile and context",
            original_length=len(adaptation_request.content),
            adapted_length=len(adapted_content)
        )

    # Original methods preserved for compatibility
    async def personalize_textbook(self, textbook: Textbook, user_profile: UserProfile) -> Textbook:
        """
        Personalize textbook content based on user profile.
        """
        personalized_textbook = textbook.copy()
        personalized_textbook.personalization_enabled = True

        # Personalize each chapter
        for chapter in personalized_textbook.chapters:
            chapter.personalized_content = await self.personalize_chapter(chapter, user_profile)

        return personalized_textbook

    async def personalize_chapter(self, chapter: Chapter, user_profile: UserProfile) -> Dict[str, Any]:
        """
        Personalize a chapter based on user profile.
        """
        # Generate personalized content based on user's background
        personalized_content = await self._generate_personalized_content(
            original_content=chapter.title,
            user_profile=user_profile,
            content_type="chapter_title"
        )

        return {
            "title": personalized_content,
            "sections": {}
        }

    async def personalize_section(self, section: Section, user_profile: UserProfile) -> Dict[str, Any]:
        """
        Personalize a section based on user profile.
        """
        # Generate personalized content based on user's background
        personalized_content = await self._generate_personalized_content(
            original_content=section.content,
            user_profile=user_profile,
            content_type="section_content"
        )

        difficulty_adjustment = self._calculate_difficulty_adjustment(user_profile)

        return {
            "content": personalized_content,
            "difficulty_level": difficulty_adjustment
        }

    async def _generate_personalized_content(self, original_content: str,
                                          user_profile: UserProfile,
                                          content_type: str) -> str:
        """
        Generate personalized content based on user profile.
        """
        # Create a prompt for personalization based on user's background
        prompt = self._create_personalization_prompt(
            original_content,
            user_profile,
            content_type
        )

        # Use OpenAI to generate personalized content
        personalized_content = self.openai_client.generate_personalized_content(
            prompt=prompt,
            user_software_background=user_profile.software_background,
            user_hardware_background=user_profile.hardware_background
        )

        return personalized_content

    def _create_personalization_prompt(self, original_content: str,
                                     user_profile: UserProfile,
                                     content_type: str) -> str:
        """
        Create a prompt for content personalization.
        """
        prompt = f"""
        Original content: {original_content}

        User Profile:
        - Software Background: {user_profile.software_background}
        - Hardware Background: {user_profile.hardware_background}
        - Programming Languages: {', '.join(user_profile.programming_languages)}
        - Hardware Experience: {', '.join(user_profile.hardware_experience)}
        - Robotics Experience: {user_profile.robotics_experience}
        - Education Level: {user_profile.education_level}

        Please adapt the above content to match the user's background level.
        For content type '{content_type}', adjust complexity, examples, and explanations
        to be appropriate for the user's experience level.
        """

        return prompt

    def _calculate_difficulty_adjustment(self, user_profile: UserProfile) -> str:
        """
        Calculate difficulty adjustment based on user profile.
        """
        # Simple mapping of user experience to difficulty level
        if user_profile.robotics_experience == "advanced" and user_profile.software_background == "advanced":
            return "advanced"
        elif user_profile.robotics_experience in ["intermediate", "advanced"] or user_profile.software_background == "intermediate":
            return "intermediate"
        else:
            return "beginner"

    async def get_personalized_content_old(self, content_id: str, user_profile: UserProfile,
                                     content_type: str) -> Optional[str]:
        """
        Get personalized content for a specific content item.
        """
        # In a real implementation, this would retrieve and personalize specific content
        # For now, return None to indicate this needs to be implemented
        return None

    def execute(self, *args, **kwargs):
        """
        Execute the service operation (required by BaseService).
        """
        # This method should be implemented based on specific personalization needs
        raise NotImplementedError("Execute method should be implemented by specific operations")