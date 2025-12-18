import os
from typing import Dict, List, Optional
from dotenv import load_dotenv
from .google_client import GoogleClient

# Load environment variables
load_dotenv()

class OpenAIClient:
    """
    Client for interacting with Google API for chat and content generation.
    """

    def __init__(self):
        # Initialize Google client as primary method
        try:
            self.google_client = GoogleClient()
            print("Google API client initialized successfully.")
        except ValueError as e:
            print(f"Google API client initialization failed: {str(e)}")
            self.google_client = None

        # Check if OpenAI API key is available (for future use)
        self.openai_api_key = os.getenv("OPENAI_API_KEY")
        if self.openai_api_key:
            self.openai_enabled = True
            print("OpenAI API key found.")
        else:
            self.openai_enabled = False
            print("OpenAI API key not found. Using Google API for all operations.")

    def generate_textbook_content(
        self,
        subject: str,
        difficulty: str,
        chapter_count: int = 5,
        target_audience: Optional[str] = None,
        include_quizzes: bool = True,
        include_summaries: bool = True
    ) -> str:
        """
        Generate textbook content using Google API.

        Args:
            subject: Subject area of the textbook
            difficulty: Difficulty level (beginner, intermediate, advanced)
            chapter_count: Number of chapters to generate
            target_audience: Description of target audience
            include_quizzes: Whether to include quizzes
            include_summaries: Whether to include summaries

        Returns:
            Generated textbook content as a string
        """
        return self.google_client.generate_textbook_content(
            subject=subject,
            difficulty=difficulty,
            chapter_count=chapter_count,
            target_audience=target_audience,
            include_quizzes=include_quizzes,
            include_summaries=include_summaries
        )

    def generate_chapter_content(
        self,
        chapter_title: str,
        subject: str,
        difficulty: str,
        position: int,
        target_audience: Optional[str] = None
    ) -> str:
        """
        Generate content for a specific chapter using Google API.

        Args:
            chapter_title: Title of the chapter
            subject: Subject area
            difficulty: Difficulty level
            position: Chapter position in the textbook
            target_audience: Description of target audience

        Returns:
            Generated chapter content as a string
        """
        # For now, create a prompt and use the textbook content generator
        # In a real implementation, this would use Google's generative AI services
        full_subject = f"{subject} - Chapter {position}: {chapter_title}"
        return self.google_client.generate_textbook_content(
            subject=full_subject,
            difficulty=difficulty,
            chapter_count=1,
            target_audience=target_audience,
            include_quizzes=False,
            include_summaries=False
        )

    def generate_interactive_element(
        self,
        element_type: str,
        content_context: str,
        difficulty: str
    ) -> str:
        """
        Generate interactive elements like quizzes or summaries using Google API.

        Args:
            element_type: Type of interactive element (quiz, summary, booster, exercise)
            content_context: Context from which to generate the element
            difficulty: Difficulty level

        Returns:
            Generated interactive element content as a string
        """
        # For now, use the textbook content generator with specific context
        # In a real implementation, this would use Google's generative AI services
        subject = f"{element_type} based on: {content_context[:50]}..."
        return self.google_client.generate_textbook_content(
            subject=subject,
            difficulty=difficulty,
            chapter_count=1,
            include_quizzes=False,
            include_summaries=False
        )

    def _build_textbook_prompt(
        self,
        subject: str,
        difficulty: str,
        chapter_count: int,
        target_audience: Optional[str],
        include_quizzes: bool,
        include_summaries: bool
    ) -> str:
        """Build prompt for textbook generation."""
        prompt = f"""
        Create a comprehensive textbook on {subject} with {chapter_count} chapters. The difficulty level is {difficulty}.
        """

        if target_audience:
            prompt += f" The target audience is: {target_audience}. "

        prompt += f"""
        Structure the textbook with:
        - Clear learning objectives for each chapter
        - Well-organized sections with appropriate headings
        - Key terms defined throughout
        - Appropriate examples and explanations for the difficulty level

        Include {'quizzes and summaries' if include_quizzes and include_summaries else 'summaries' if include_summaries else 'quizzes' if include_quizzes else 'no interactive elements'}.

        Format the response in a structured way that can be parsed for textbook generation.
        """

        return prompt

    def _build_chapter_prompt(
        self,
        chapter_title: str,
        subject: str,
        difficulty: str,
        position: int,
        target_audience: Optional[str]
    ) -> str:
        """Build prompt for chapter generation."""
        prompt = f"""
        Create detailed content for Chapter {position}: {chapter_title} in a textbook on {subject}.
        The difficulty level is {difficulty}.
        """

        if target_audience:
            prompt += f" The target audience is: {target_audience}. "

        prompt += """
        Include:
        - Learning objectives at the beginning
        - Well-organized sections with appropriate headings
        - Key terms defined throughout
        - Appropriate examples and explanations for the difficulty level
        - A brief summary at the end

        Make the content comprehensive but focused on the chapter title.
        """

        return prompt

    def _build_interactive_prompt(
        self,
        element_type: str,
        content_context: str,
        difficulty: str
    ) -> str:
        """Build prompt for interactive element generation."""
        if element_type == "quiz":
            prompt = f"""
            Create a quiz based on the following content context: {content_context}
            The difficulty level is {difficulty}.
            Include multiple choice questions with 4 options each and indicate the correct answer.
            """
        elif element_type == "summary":
            prompt = f"""
            Create a concise summary of the following content: {content_context}
            The difficulty level is {difficulty}.
            Focus on the key points and learning objectives.
            """
        elif element_type == "booster":
            prompt = f"""
            Create a learning booster based on the following content: {content_context}
            The difficulty level is {difficulty}.
            This could be additional practice problems, discussion questions, or extension activities.
            """
        elif element_type == "exercise":
            prompt = f"""
            Create an exercise based on the following content: {content_context}
            The difficulty level is {difficulty}.
            This should be a practical application of the concepts covered.
            """
        else:
            prompt = f"""
            Create an interactive element of type {element_type} based on the following content: {content_context}
            The difficulty level is {difficulty}.
            """

        return prompt

    def generate_chat_response(self, query: str, context: str = "") -> str:
        """
        Generate a chat response using Google API with provided context.

        Args:
            query: User's query/question
            context: Context from textbook content to base response on

        Returns:
            Generated response as a string
        """
        # Use Google API for chat responses
        if self.google_client:
            try:
                # Prepare the full prompt with context
                system_prompt = "You are an AI assistant for a Physical AI and Humanoid Robotics textbook. Answer questions based on the provided context from the textbook."

                full_query = query
                if context:
                    full_query = f"Context from textbook:\n{context}\n\nQuestion: {query}\n\nPlease provide a helpful, accurate answer based on the textbook content. If the context doesn't contain the answer, acknowledge this and suggest where they might find the information."

                return self.google_client.generate_textbook_content(
                    subject=full_query,
                    difficulty="intermediate",
                    chapter_count=1,
                    include_quizzes=False,
                    include_summaries=False
                )
            except Exception as e:
                print(f"Google API error: {str(e)}")
                return f"I'm sorry, I encountered an error processing your request: {str(e)}"
        else:
            return "I'm sorry, but I'm unable to process your request as Google API is not configured properly."