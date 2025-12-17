from typing import Dict, Any, List, Optional
from datetime import datetime
import logging
from dataclasses import dataclass

from ...models.textbook import Textbook
from ...models.chapter import Chapter
from ...models.section import Section
from ...models.user_profile import UserProfile
from ...schemas.personalization_schemas import (
    ContentAdaptationRequest,
    ContentAdaptationResponse
)


logger = logging.getLogger(__name__)


@dataclass
class AdaptationResult:
    """
    Result of content adaptation
    """
    original_content: str
    adapted_content: str
    adaptation_type: str
    confidence_score: float
    metadata: Dict[str, Any]


class ContentAdapter:
    """
    Service for adapting content based on user profile and context
    """

    def __init__(self):
        self.adaptation_history = {}  # For tracking adaptation effectiveness

    def adapt_textbook_content(
        self,
        textbook: Textbook,
        user_profile: UserProfile,
        adaptation_preferences: Optional[Dict[str, Any]] = None
    ) -> Textbook:
        """
        Adapt entire textbook content based on user profile
        """
        adapted_textbook = textbook.copy()
        adapted_textbook.personalization_enabled = True

        # Adapt each chapter
        for i, chapter in enumerate(adapted_textbook.chapters):
            adapted_chapter = self.adapt_chapter_content(
                chapter,
                user_profile,
                adaptation_preferences
            )
            adapted_textbook.chapters[i] = adapted_chapter

        return adapted_textbook

    def adapt_chapter_content(
        self,
        chapter: Chapter,
        user_profile: UserProfile,
        adaptation_preferences: Optional[Dict[str, Any]] = None
    ) -> Chapter:
        """
        Adapt chapter content based on user profile
        """
        adapted_chapter = chapter.copy()

        # Adapt chapter title if needed
        if chapter.title:
            title_adaptation = self.adapt_content(
                chapter.title,
                user_profile,
                "chapter_title",
                adaptation_preferences
            )
            adapted_chapter.title = title_adaptation.adapted_content

        # Adapt chapter content if exists
        if chapter.content:
            content_adaptation = self.adapt_content(
                chapter.content,
                user_profile,
                "chapter_content",
                adaptation_preferences
            )
            adapted_chapter.content = content_adaptation.adapted_content

        # Adapt sections if they exist
        if hasattr(chapter, 'sections') and chapter.sections:
            adapted_sections = []
            for section in chapter.sections:
                adapted_section = self.adapt_section_content(
                    section,
                    user_profile,
                    adaptation_preferences
                )
                adapted_sections.append(adapted_section)
            adapted_chapter.sections = adapted_sections

        return adapted_chapter

    def adapt_section_content(
        self,
        section: Section,
        user_profile: UserProfile,
        adaptation_preferences: Optional[Dict[str, Any]] = None
    ) -> Section:
        """
        Adapt section content based on user profile
        """
        adapted_section = section.copy()

        # Adapt section title if needed
        if section.title:
            title_adaptation = self.adapt_content(
                section.title,
                user_profile,
                "section_title",
                adaptation_preferences
            )
            adapted_section.title = title_adaptation.adapted_content

        # Adapt section content
        if section.content:
            content_adaptation = self.adapt_content(
                section.content,
                user_profile,
                "section_content",
                adaptation_preferences
            )
            adapted_section.content = content_adaptation.adapted_content

        return adapted_section

    def adapt_content(
        self,
        original_content: str,
        user_profile: UserProfile,
        content_type: str = "general",
        adaptation_preferences: Optional[Dict[str, Any]] = None
    ) -> AdaptationResult:
        """
        Adapt specific content based on user profile and preferences
        """
        try:
            # Determine adaptation strategy based on user profile
            adaptation_strategy = self._determine_adaptation_strategy(
                user_profile,
                content_type,
                adaptation_preferences
            )

            # Apply adaptations
            adapted_content = original_content

            # Apply difficulty adaptation
            if adaptation_strategy.get("adjust_difficulty"):
                adapted_content = self._adjust_difficulty(
                    adapted_content,
                    user_profile,
                    adaptation_strategy.get("difficulty_level", "intermediate")
                )

            # Apply example adaptation
            if adaptation_strategy.get("adjust_examples"):
                adapted_content = self._adjust_examples(
                    adapted_content,
                    user_profile
                )

            # Apply explanation style adaptation
            if adaptation_strategy.get("adjust_explanation_style"):
                adapted_content = self._adjust_explanation_style(
                    adapted_content,
                    user_profile
                )

            # Apply language complexity adaptation
            if adaptation_strategy.get("adjust_language"):
                adapted_content = self._adjust_language_complexity(
                    adapted_content,
                    user_profile
                )

            # Calculate confidence score
            confidence_score = self._calculate_adaptation_confidence(
                user_profile,
                adaptation_strategy
            )

            return AdaptationResult(
                original_content=original_content,
                adapted_content=adapted_content,
                adaptation_type=content_type,
                confidence_score=confidence_score,
                metadata={
                    "strategy": adaptation_strategy,
                    "timestamp": datetime.now(),
                    "user_profile_match": self._calculate_profile_match(user_profile, content_type)
                }
            )

        except Exception as e:
            logger.error(f"Error adapting content: {str(e)}")
            # Return original content if adaptation fails
            return AdaptationResult(
                original_content=original_content,
                adapted_content=original_content,
                adaptation_type=content_type,
                confidence_score=0.0,
                metadata={
                    "error": str(e),
                    "timestamp": datetime.now()
                }
            )

    def _determine_adaptation_strategy(
        self,
        user_profile: UserProfile,
        content_type: str,
        adaptation_preferences: Optional[Dict[str, Any]] = None
    ) -> Dict[str, Any]:
        """
        Determine adaptation strategy based on user profile
        """
        strategy = {
            "adjust_difficulty": True,
            "adjust_examples": True,
            "adjust_explanation_style": True,
            "adjust_language": True,
            "difficulty_level": self._get_difficulty_level(user_profile),
            "example_preference": self._get_example_preference(user_profile),
            "explanation_style": self._get_explanation_style(user_profile)
        }

        # Override with preferences if provided
        if adaptation_preferences:
            for key, value in adaptation_preferences.items():
                if key in strategy:
                    strategy[key] = value

        return strategy

    def _get_difficulty_level(self, user_profile: UserProfile) -> str:
        """
        Get appropriate difficulty level based on user profile
        """
        # Calculate average experience level
        exp_levels = [
            self._map_experience_to_level(user_profile.software_background),
            self._map_experience_to_level(user_profile.hardware_background),
            self._map_experience_to_level(user_profile.robotics_experience)
        ]

        avg_level = sum(exp_levels) / len(exp_levels) if exp_levels else 1

        if avg_level >= 2.5:
            return "advanced"
        elif avg_level >= 1.5:
            return "intermediate"
        else:
            return "beginner"

    def _get_example_preference(self, user_profile: UserProfile) -> str:
        """
        Get example preference based on user profile
        """
        if "Python" in user_profile.programming_languages or "ROS" in user_profile.programming_languages:
            return "code_implementation"
        elif "MATLAB" in user_profile.programming_languages:
            return "mathematical_modeling"
        elif any(platform in ["ROSbot", "TurtleBot", "NVIDIA Isaac"] for platform in user_profile.hardware_experience):
            return "practical_implementation"
        else:
            return "conceptual_explanation"

    def _get_explanation_style(self, user_profile: UserProfile) -> str:
        """
        Get explanation style based on user profile
        """
        if user_profile.math_background == "advanced" and user_profile.robotics_experience in ["intermediate", "advanced"]:
            return "mathematical_formal"
        elif user_profile.math_background == "basic" or user_profile.robotics_experience == "none":
            return "intuitive_visual"
        else:
            return "balanced_approach"

    def _map_experience_to_level(self, experience: str) -> float:
        """
        Map experience level string to numeric value
        """
        mapping = {
            "beginner": 1,
            "basic": 1,
            "intermediate": 2,
            "advanced": 3,
            "none": 0
        }
        return mapping.get(experience, 1)

    def _adjust_difficulty(
        self,
        content: str,
        user_profile: UserProfile,
        target_difficulty: str
    ) -> str:
        """
        Adjust content difficulty based on target level
        """
        if target_difficulty == "beginner":
            return self._make_beginner_friendly(content)
        elif target_difficulty == "advanced":
            return self._make_advanced(content)
        else:  # intermediate
            return self._make_intermediate(content)

    def _make_beginner_friendly(self, content: str) -> str:
        """
        Make content more beginner-friendly
        """
        # Replace complex terms with simpler ones
        beginner_content = content.replace("algorithm", "method")
        beginner_content = beginner_content.replace("implementation", "way to do it")
        beginner_content = beginner_content.replace("optimization", "improvement")
        beginner_content = beginner_content.replace("complexity", "difficulty")

        # Add explanations and examples
        if "method" in beginner_content.lower():
            beginner_content += "\n\nThis method works by following these simple steps:\n1. First, we do this\n2. Then, we do that\n3. Finally, we get the result"

        return beginner_content

    def _make_advanced(self, content: str) -> str:
        """
        Make content more advanced
        """
        # Replace simple terms with more complex ones
        advanced_content = content.replace("method", "algorithm")
        advanced_content = advanced_content.replace("way to do it", "implementation")
        advanced_content = advanced_content.replace("improvement", "optimization")
        advanced_content = advanced_content.replace("difficulty", "complexity")

        # Add technical depth
        if "algorithm" in advanced_content.lower():
            advanced_content += "\n\nImplementation considerations:\n- Time complexity: O(n log n)\n- Space complexity: O(n)\n- Performance optimizations: caching, pruning"

        return advanced_content

    def _make_intermediate(self, content: str) -> str:
        """
        Make content intermediate level
        """
        # Keep content at moderate complexity
        intermediate_content = content.replace("simple steps", "steps")
        intermediate_content = intermediate_content.replace("way to do it", "approach")

        # Add moderate technical details
        if "approach" in intermediate_content.lower():
            intermediate_content += "\n\nImplementation details:\n- Key considerations\n- Best practices\n- Common pitfalls to avoid"

        return intermediate_content

    def _adjust_examples(self, content: str, user_profile: UserProfile) -> str:
        """
        Adjust examples based on user profile
        """
        example_preference = self._get_example_preference(user_profile)

        if example_preference == "code_implementation":
            return content + "\n\nExample:\n```python\n# Implementation code here\n```\n"
        elif example_preference == "mathematical_modeling":
            return content + "\n\nMathematical representation:\nLet x be the variable where f(x) = ...\n"
        elif example_preference == "practical_implementation":
            return content + "\n\nPractical example:\nConnect sensor to pin 1, configure for I2C communication\n"
        else:  # conceptual_explanation
            return content + "\n\nFor example, think of it as...\n"

        return content

    def _adjust_explanation_style(self, content: str, user_profile: UserProfile) -> str:
        """
        Adjust explanation style based on user profile
        """
        style = self._get_explanation_style(user_profile)

        if style == "mathematical_formal":
            return content + "\n\nFormal definition: Let S be a set such that..."
        elif style == "intuitive_visual":
            return content + "\n\nImagine this as..."
        else:  # balanced_approach
            return content + "\n\nBoth conceptually and formally..."

    def _adjust_language_complexity(self, content: str, user_profile: UserProfile) -> str:
        """
        Adjust language complexity based on user profile
        """
        difficulty_level = self._get_difficulty_level(user_profile)

        if difficulty_level == "beginner":
            # Use simpler vocabulary and shorter sentences
            simple_content = content.replace("utilize", "use")
            simple_content = simple_content.replace("demonstrate", "show")
            simple_content = simple_content.replace("implement", "create")
            return simple_content
        elif difficulty_level == "advanced":
            # Use more technical vocabulary
            advanced_content = content.replace("use", "utilize")
            advanced_content = advanced_content.replace("show", "demonstrate")
            advanced_content = advanced_content.replace("create", "implement")
            return advanced_content
        else:
            # Keep moderate complexity
            return content

    def _calculate_adaptation_confidence(self, user_profile: UserProfile, strategy: Dict[str, Any]) -> float:
        """
        Calculate confidence in adaptation based on profile completeness
        """
        completeness_score = 0
        total_fields = 0

        # Check various profile fields
        fields_to_check = [
            user_profile.software_background,
            user_profile.hardware_background,
            user_profile.robotics_experience,
            user_profile.math_background
        ]

        for field in fields_to_check:
            if field:
                completeness_score += 1
            total_fields += 1

        # Check list fields
        list_fields = [user_profile.programming_languages, user_profile.hardware_experience]
        for field in list_fields:
            if field and len(field) > 0:
                completeness_score += 1
            total_fields += 1

        if total_fields == 0:
            return 0.3  # Default low confidence

        return min(0.95, completeness_score / total_fields)

    def _calculate_profile_match(self, user_profile: UserProfile, content_type: str) -> float:
        """
        Calculate how well the content matches the user profile
        """
        # This is a simplified calculation
        # In a real implementation, this would be more sophisticated
        return 0.7

    def record_adaptation_effectiveness(
        self,
        user_id: str,
        content_id: str,
        original_content: str,
        adapted_content: str,
        user_feedback: Optional[Dict[str, Any]] = None
    ) -> bool:
        """
        Record adaptation effectiveness for future improvements
        """
        try:
            adaptation_id = f"{user_id}_{content_id}_{datetime.now().timestamp()}"

            self.adaptation_history[adaptation_id] = {
                "user_id": user_id,
                "content_id": content_id,
                "original_content_length": len(original_content),
                "adapted_content_length": len(adapted_content),
                "timestamp": datetime.now(),
                "user_feedback": user_feedback or {}
            }

            return True
        except Exception as e:
            logger.error(f"Error recording adaptation effectiveness: {str(e)}")
            return False

    def get_adaptation_suggestions(
        self,
        user_profile: UserProfile,
        content_type: str = "general"
    ) -> List[Dict[str, str]]:
        """
        Get suggestions for content adaptation based on user profile
        """
        suggestions = []

        difficulty_level = self._get_difficulty_level(user_profile)
        suggestions.append({
            "type": "difficulty",
            "suggestion": f"Content should be adapted to {difficulty_level} level",
            "priority": "high"
        })

        example_preference = self._get_example_preference(user_profile)
        suggestions.append({
            "type": "examples",
            "suggestion": f"Examples should focus on {example_preference}",
            "priority": "medium"
        })

        explanation_style = self._get_explanation_style(user_profile)
        suggestions.append({
            "type": "explanation",
            "suggestion": f"Explanation style should be {explanation_style}",
            "priority": "medium"
        })

        return suggestions

    async def adapt_content_request(
        self,
        adaptation_request: ContentAdaptationRequest
    ) -> ContentAdaptationResponse:
        """
        Adapt content based on adaptation request
        """
        # Extract user profile from context
        user_profile_data = adaptation_request.context.get('user_profile', {})

        # Create a minimal user profile object for adaptation
        user_profile = UserProfile(
            id=adaptation_request.context.get('user_id', 'unknown'),
            email="unknown@example.com",  # Placeholder
            name="Unknown User",  # Placeholder
            software_background=user_profile_data.get('software_background', 'beginner'),
            hardware_background=user_profile_data.get('hardware_background', 'none'),
            programming_languages=user_profile_data.get('programming_languages', []),
            hardware_experience=user_profile_data.get('hardware_experience', []),
            robotics_experience=user_profile_data.get('robotics_experience', 'none'),
            education_level=user_profile_data.get('education_level', 'undergraduate')
        )

        # Perform adaptation
        result = self.adapt_content(
            adaptation_request.content,
            user_profile,
            adaptation_request.context.get('content_type', 'general'),
            adaptation_request.adaptation_preferences
        )

        return ContentAdaptationResponse(
            original_content=adaptation_request.content,
            adapted_content=result.adapted_content,
            adaptation_type="profile_based",
            adaptation_reason="Content adapted based on user profile and preferences",
            original_length=len(adaptation_request.content),
            adapted_length=len(result.adapted_content)
        )