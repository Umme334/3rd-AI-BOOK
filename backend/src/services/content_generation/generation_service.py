import asyncio
from typing import Dict, Any, List
from datetime import datetime

from ...models.textbook import Textbook, TextbookStatus
from ...models.chapter import Chapter
from ...models.section import Section
from ...models.interactive_element import InteractiveElement
from ...services.openai_client import OpenAIClient
from ...services.textbook_service import TextbookService
from ...utils.storage import FileStorage
from ...exceptions import ContentGenerationError, TextbookNotFoundError
from ..base_service import BaseService


class ContentGenerationService(BaseService):
    """
    Service for generating Physical AI and Humanoid Robotics textbook content using AI.
    """

    def __init__(self):
        super().__init__()
        self.openai_client = OpenAIClient()
        self.textbook_service = TextbookService()
        self.storage = FileStorage()

    async def generate_textbook_content(self, textbook_id: str) -> Textbook:
        """
        Generate complete Physical AI and Humanoid Robotics textbook content using AI.

        Args:
            textbook_id: Unique identifier of the textbook to generate content for

        Returns:
            Updated textbook object with generated content
        """
        try:
            # Get the textbook to generate content for
            textbook = self.textbook_service.get_textbook(textbook_id)

            # Update status to generating
            self.textbook_service.update_textbook_status(textbook_id, TextbookStatus.generating)

            # Define Physical AI course modules based on the hackathon requirements
            if "Physical AI" in textbook.subject or "Humanoid Robotics" in textbook.subject:
                modules = [
                    "Introduction to Physical AI and Embodied Intelligence",
                    "ROS 2: The Robotic Nervous System",
                    "Robot Simulation with Gazebo",
                    "NVIDIA Isaac Platform",
                    "Vision-Language-Action Integration"
                ]
            else:
                modules = [f"Module {i+1}: [To be determined based on subject]" for i in range(5)]

            # Generate chapters
            chapters = []
            for i in range(1, textbook.chapter_count + 1):
                if i <= len(modules):
                    chapter_title = modules[i-1]
                else:
                    chapter_title = f"Chapter {i}: [To be determined based on subject]"

                chapter_content = self.openai_client.generate_chapter_content(
                    chapter_title=chapter_title,
                    subject=textbook.subject,
                    difficulty=textbook.difficulty,
                    position=i,
                    target_audience=textbook.target_audience
                )

                # Determine module type based on the chapter title
                module_type = self._determine_module_type(chapter_title)

                # Create a basic chapter structure
                chapter = Chapter(
                    id=f"ch-{textbook_id}-{i}",
                    title=chapter_title,
                    position=i,
                    sections=[],
                    word_count=len(chapter_content.split()),
                    learning_objectives=[],
                    # Physical AI specific attributes
                    module_type=module_type,
                    hardware_focus=self._get_hardware_focus(module_type),
                    difficulty_level=textbook.difficulty,
                    estimated_duration=self._get_estimated_duration(module_type),
                    prerequisites=self._get_prerequisites(module_type)
                )

                # Generate sections for the chapter
                sections = await self._generate_sections_for_chapter(
                    chapter_content, textbook.difficulty, i, module_type
                )
                chapter.sections = sections

                chapters.append(chapter)

            # Set learning outcomes for the textbook
            textbook.learning_outcomes = self._get_learning_outcomes(textbook.subject)
            textbook.hardware_requirements = self._get_hardware_requirements(textbook.subject)

            # Update the textbook with generated content
            textbook.chapters = chapters
            textbook.status = TextbookStatus.complete
            textbook.updated_at = datetime.now()

            # Save the updated textbook
            textbook_data = textbook.model_dump()
            self.storage.save_textbook(textbook_id, textbook_data)

            return textbook

        except Exception as e:
            # Update status to failed if generation fails
            self.textbook_service.update_textbook_status(textbook_id, TextbookStatus.failed)
            raise ContentGenerationError(f"Failed to generate textbook content: {str(e)}")

    def _determine_module_type(self, chapter_title: str) -> str:
        """
        Determine the Physical AI module type based on chapter title.
        """
        chapter_lower = chapter_title.lower()

        if "ros" in chapter_lower or "nervous system" in chapter_lower:
            return "ROS 2"
        elif "gazebo" in chapter_lower or "simulation" in chapter_lower:
            return "Gazebo"
        elif "nvidia" in chapter_lower or "isaac" in chapter_lower:
            return "NVIDIA Isaac"
        elif "vision" in chapter_lower or "language" in chapter_lower or "action" in chapter_lower:
            return "Vision-Language-Action"
        elif "introduction" in chapter_lower or "embodied" in chapter_lower:
            return "Introduction"
        else:
            return "General"

    def _get_hardware_focus(self, module_type: str) -> List[str]:
        """
        Get hardware focus for a specific module type.
        """
        hardware_mapping = {
            "ROS 2": ["simulation", "real-hardware"],
            "Gazebo": ["simulation"],
            "NVIDIA Isaac": ["simulation", "real-hardware", "cloud-deployment"],
            "Vision-Language-Action": ["simulation", "real-hardware"],
            "Introduction": ["theoretical"],
            "General": ["theoretical"]
        }
        return hardware_mapping.get(module_type, ["theoretical"])

    def _get_estimated_duration(self, module_type: str) -> str:
        """
        Get estimated duration for a specific module type.
        """
        duration_mapping = {
            "ROS 2": "2-3 weeks",
            "Gazebo": "2 weeks",
            "NVIDIA Isaac": "3-4 weeks",
            "Vision-Language-Action": "2 weeks",
            "Introduction": "1 week",
            "General": "1-2 weeks"
        }
        return duration_mapping.get(module_type, "1-2 weeks")

    def _get_prerequisites(self, module_type: str) -> List[str]:
        """
        Get prerequisites for a specific module type.
        """
        prereq_mapping = {
            "ROS 2": ["Python basics", "Linux command line"],
            "Gazebo": ["Python basics", "Linux command line", "Basic ROS concepts"],
            "NVIDIA Isaac": ["Python basics", "Linux command line", "ROS 2 basics", "CUDA basics"],
            "Vision-Language-Action": ["Python basics", "Linux command line", "ROS 2 basics", "Machine Learning basics"],
            "Introduction": ["Basic programming concepts"],
            "General": ["Basic programming concepts"]
        }
        return prereq_mapping.get(module_type, ["Basic programming concepts"])

    def _get_learning_outcomes(self, subject: str) -> List[str]:
        """
        Get learning outcomes for the Physical AI textbook.
        """
        if "Physical AI" in subject or "Humanoid Robotics" in subject:
            return [
                "Understand Physical AI principles and embodied intelligence",
                "Master ROS 2 (Robot Operating System) for robotic control",
                "Simulate robots with Gazebo and Unity",
                "Develop with NVIDIA Isaac AI robot platform",
                "Design humanoid robots for natural interactions",
                "Integrate GPT models for conversational robotics"
            ]
        else:
            return []

    def _get_hardware_requirements(self, subject: str) -> List[str]:
        """
        Get hardware requirements for the Physical AI textbook.
        """
        if "Physical AI" in subject or "Humanoid Robotics" in subject:
            return [
                "NVIDIA RTX 4070 Ti (12GB VRAM) or higher",
                "Intel Core i7 (13th Gen+) or AMD Ryzen 9",
                "64 GB DDR5 RAM",
                "Ubuntu 22.04 LTS",
                "NVIDIA Jetson Orin Nano (8GB) or Orin NX (16GB)",
                "Intel RealSense D435i or D455",
                "USB Microphone/Speaker array (e.g., ReSpeaker)"
            ]
        else:
            return []

    async def _generate_sections_for_chapter(
        self, chapter_content: str, difficulty: str, chapter_position: int, module_type: str
    ) -> List['Section']:
        """
        Generate sections for a chapter with Physical AI focus.

        Args:
            chapter_content: Raw content for the chapter
            difficulty: Difficulty level
            chapter_position: Position of the chapter
            module_type: Type of Physical AI module

        Returns:
            List of section objects
        """
        # For now, we'll create a single section with the chapter content
        # In a real implementation, this would parse the content into sections
        section = Section(
            id=f"sec-{chapter_position}-1",
            title=f"Section 1",
            content=chapter_content,
            position=1,
            interactive_elements=[],
            key_terms=[],
            # Physical AI specific attributes
            topic_category=self._get_topic_category(module_type),
            hardware_requirements=self._get_section_hardware_requirements(module_type),
            code_examples=[]
        )

        # Add interactive elements
        interactive_elements = await self._generate_interactive_elements(
            chapter_content, difficulty, module_type
        )
        section.interactive_elements = interactive_elements

        # Extract key terms (in a real implementation, this would use NLP)
        section.key_terms = self._extract_key_terms(chapter_content)

        return [section]

    def _get_topic_category(self, module_type: str) -> str:
        """
        Get topic category based on module type.
        """
        category_mapping = {
            "ROS 2": "theoretical",
            "Gazebo": "simulation",
            "NVIDIA Isaac": "practical",
            "Vision-Language-Action": "practical",
            "Introduction": "theoretical",
            "General": "theoretical"
        }
        return category_mapping.get(module_type, "theoretical")

    def _get_section_hardware_requirements(self, module_type: str) -> List[str]:
        """
        Get hardware requirements for a specific section based on module type.
        """
        hw_mapping = {
            "ROS 2": ["Computer with ROS 2 installed"],
            "Gazebo": ["Computer with Gazebo installed", "NVIDIA GPU recommended"],
            "NVIDIA Isaac": ["NVIDIA RTX GPU", "Isaac Sim installed"],
            "Vision-Language-Action": ["Camera or simulated sensor"],
            "Introduction": [],
            "General": []
        }
        return hw_mapping.get(module_type, [])

    async def _generate_interactive_elements(
        self, content_context: str, difficulty: str, module_type: str
    ) -> List['InteractiveElement']:
        """
        Generate interactive elements for Physical AI content.

        Args:
            content_context: Content to generate elements for
            difficulty: Difficulty level
            module_type: Type of Physical AI module

        Returns:
            List of interactive element objects
        """
        elements = []

        # Generate a quiz
        quiz_content = self.openai_client.generate_interactive_element(
            element_type="quiz",
            content_context=content_context,
            difficulty=difficulty
        )
        quiz_element = InteractiveElement(
            id=f"ie-quiz-{len(elements) + 1}",
            type="quiz",
            content=quiz_content,
            position=len(elements) + 1,
            # Physical AI specific attributes
            hardware_relevance=self._get_interactive_hardware_relevance(module_type),
            difficulty_level=difficulty,
            learning_objective="Assess understanding of key concepts"
        )
        elements.append(quiz_element)

        # Generate a summary
        summary_content = self.openai_client.generate_interactive_element(
            element_type="summary",
            content_context=content_context,
            difficulty=difficulty
        )
        summary_element = InteractiveElement(
            id=f"ie-summary-{len(elements) + 1}",
            type="summary",
            content=summary_content,
            position=len(elements) + 1,
            # Physical AI specific attributes
            hardware_relevance=self._get_interactive_hardware_relevance(module_type),
            difficulty_level=difficulty,
            learning_objective="Summarize key concepts and takeaways"
        )
        elements.append(summary_element)

        # For Physical AI modules, add simulation or practical elements
        if module_type in ["Gazebo", "NVIDIA Isaac", "Vision-Language-Action"]:
            practical_content = self.openai_client.generate_interactive_element(
                element_type="exercise",
                content_context=content_context,
                difficulty=difficulty
            )
            practical_element = InteractiveElement(
                id=f"ie-practical-{len(elements) + 1}",
                type="exercise",
                content=practical_content,
                position=len(elements) + 1,
                # Physical AI specific attributes
                hardware_relevance=["simulation"] if module_type in ["Gazebo", "NVIDIA Isaac"] else ["real-robot"],
                difficulty_level=difficulty,
                learning_objective="Apply concepts in practical scenario"
            )
            elements.append(practical_element)

        return elements

    def _get_interactive_hardware_relevance(self, module_type: str) -> List[str]:
        """
        Get hardware relevance for interactive elements based on module type.
        """
        hw_mapping = {
            "ROS 2": ["simulation", "real-hardware"],
            "Gazebo": ["simulation"],
            "NVIDIA Isaac": ["simulation", "real-hardware"],
            "Vision-Language-Action": ["simulation", "real-hardware"],
            "Introduction": ["theoretical"],
            "General": ["theoretical"]
        }
        return hw_mapping.get(module_type, ["theoretical"])

    def _extract_key_terms(self, content: str) -> List[str]:
        """
        Extract key terms from content (simplified implementation).

        Args:
            content: Content to extract terms from

        Returns:
            List of key terms
        """
        # This is a simplified implementation
        # In a real implementation, this would use NLP techniques
        words = content.split()
        # Take the first few capitalized words as potential key terms
        key_terms = []
        for word in words[:50]:  # Limit to first 50 words
            cleaned_word = word.strip('.,;:"!?()[]{}')
            if cleaned_word and cleaned_word[0].isupper() and len(cleaned_word) > 3:
                key_terms.append(cleaned_word)

        return list(set(key_terms))  # Remove duplicates

    def execute(self, *args, **kwargs):
        """
        Execute the service operation (required by BaseService).
        """
        # This method should be implemented based on specific generation needs
        raise NotImplementedError("Execute method should be implemented by specific operations")