from typing import Any, Dict
from .base_skill import BaseSkill, SkillResult
from ..services.openai_client import OpenAIClient


class TextGenerationSkill(BaseSkill):
    """Skill for generating text responses based on context"""

    def __init__(self):
        super().__init__(
            skill_id="text_generation",
            name="Text Generation",
            description="Generates text responses based on query and context using AI models"
        )
        self.openai_client = OpenAIClient()

    async def execute(self, **kwargs) -> SkillResult:
        """Execute text generation"""
        try:
            # Validate required parameters
            if "query" not in kwargs:
                return SkillResult(
                    success=False,
                    data=None,
                    message="Missing required parameter: query"
                )

            query = kwargs["query"]
            context = kwargs.get("context", "")
            selected_text = kwargs.get("selected_text", "")

            # Combine context and selected text if available
            full_context = context
            if selected_text:
                full_context = f"Selected Text: {selected_text}\n\nContext: {full_context}"

            # Generate response
            response = await self.openai_client.generate_chat_response(
                query=query,
                context=full_context
            )

            return SkillResult(
                success=True,
                data=response,
                message="Successfully generated response",
                metadata={"query_length": len(query), "context_length": len(full_context)}
            )
        except Exception as e:
            return SkillResult(
                success=False,
                data=None,
                message=f"Error during text generation: {str(e)}"
            )

    async def validate_inputs(self, **kwargs) -> bool:
        """Validate input parameters"""
        required_params = ["query"]
        for param in required_params:
            if param not in kwargs:
                return False
        return True


class TextSummarizationSkill(BaseSkill):
    """Skill for summarizing text content"""

    def __init__(self):
        super().__init__(
            skill_id="text_summarization",
            name="Text Summarization",
            description="Creates concise summaries of provided text content"
        )
        self.openai_client = OpenAIClient()

    async def execute(self, **kwargs) -> SkillResult:
        """Execute text summarization"""
        try:
            # Validate required parameters
            if "content" not in kwargs:
                return SkillResult(
                    success=False,
                    data=None,
                    message="Missing required parameter: content"
                )

            content = kwargs["content"]
            max_length = kwargs.get("max_length", 200)
            format_type = kwargs.get("format", "concise")  # concise, detailed, bullet_points

            # Create appropriate prompt based on format
            if format_type == "bullet_points":
                prompt = f"Please provide a bullet point summary of the following content in no more than {max_length} words: {content}"
            elif format_type == "detailed":
                prompt = f"Please provide a detailed summary of the following content: {content}"
            else:  # concise
                prompt = f"Please provide a concise summary of the following content in no more than {max_length} words: {content}"

            # Generate summary
            summary = await self.openai_client.generate_chat_response(
                query=prompt,
                context=""
            )

            return SkillResult(
                success=True,
                data=summary,
                message="Successfully generated summary",
                metadata={
                    "original_length": len(content),
                    "summary_length": len(summary),
                    "format": format_type
                }
            )
        except Exception as e:
            return SkillResult(
                success=False,
                data=None,
                message=f"Error during text summarization: {str(e)}"
            )

    async def validate_inputs(self, **kwargs) -> bool:
        """Validate input parameters"""
        required_params = ["content"]
        for param in required_params:
            if param not in kwargs:
                return False
        return True


class QuestionAnsweringSkill(BaseSkill):
    """Skill for answering questions based on provided context"""

    def __init__(self):
        super().__init__(
            skill_id="question_answering",
            name="Question Answering",
            description="Answers specific questions based on provided context"
        )
        self.openai_client = OpenAIClient()

    async def execute(self, **kwargs) -> SkillResult:
        """Execute question answering"""
        try:
            # Validate required parameters
            if "question" not in kwargs or "context" not in kwargs:
                return SkillResult(
                    success=False,
                    data=None,
                    message="Missing required parameters: question and context"
                )

            question = kwargs["question"]
            context = kwargs["context"]
            include_sources = kwargs.get("include_sources", False)

            # Create prompt for question answering
            prompt = f"Based on the following context, answer the question: {question}\n\nContext: {context}"

            # Generate answer
            answer = await self.openai_client.generate_chat_response(
                query=prompt,
                context=""
            )

            result_data = {"answer": answer}
            if include_sources:
                # In a real implementation, this would include source references
                result_data["sources"] = []  # Placeholder for source tracking

            return SkillResult(
                success=True,
                data=result_data,
                message="Successfully answered question",
                metadata={"question_length": len(question), "context_length": len(context)}
            )
        except Exception as e:
            return SkillResult(
                success=False,
                data=None,
                message=f"Error during question answering: {str(e)}"
            )

    async def validate_inputs(self, **kwargs) -> bool:
        """Validate input parameters"""
        required_params = ["question", "context"]
        for param in required_params:
            if param not in kwargs:
                return False
        return True