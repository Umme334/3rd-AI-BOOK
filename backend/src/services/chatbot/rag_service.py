import asyncio
from typing import List, Dict, Any, Optional
from datetime import datetime

from ...models.chatbot_session import ChatbotSession, ChatbotMessage
from ...models.textbook import Textbook
from ...models.chapter import Chapter
from ...models.section import Section
from ...services.openai_client import OpenAIClient
from ..base_service import BaseService
from .content_chunker import ContentChunker, ContentChunk
from .vector_store_service import VectorStoreService
from .query_processor import QueryProcessor
from ...agents.agent_manager import AgentManager
from ...skills.skills_manager import SkillsManager


class RAGService(BaseService):
    """
    Service for Retrieval-Augmented Generation (RAG) chatbot functionality.
    """

    def __init__(self):
        super().__init__()
        self.openai_client = OpenAIClient()
        self.content_chunker = ContentChunker()
        self.vector_store_service = VectorStoreService()
        self.query_processor = QueryProcessor()
        # Initialize agent manager for sub-agent architecture
        self.agent_manager = AgentManager()
        self.agent_manager.initialize_agents()
        # Initialize skills manager for specialized operations
        self.skills_manager = SkillsManager()

    async def create_chatbot_session(self, textbook_id: str, user_id: Optional[str] = None) -> ChatbotSession:
        """
        Create a new chatbot session for a textbook.
        """
        session_id = f"session-{textbook_id}-{datetime.now().strftime('%Y%m%d%H%M%S')}"

        session = ChatbotSession(
            id=session_id,
            textbook_id=textbook_id,
            user_id=user_id,
            messages=[],
            context_chunks=[],
            is_active=True
        )

        return session

    async def process_query(self, session: ChatbotSession, query: str, selected_text: Optional[str] = None) -> str:
        """
        Process a user query using RAG approach with sub-agent architecture.

        Args:
            session: The chatbot session
            query: The user's query
            selected_text: Optional text selected by the user to focus the query on
        """
        try:
            # Use the agent manager to process the RAG query
            result = await self.agent_manager.process_rag_query(
                query=query,
                textbook_id=session.textbook_id,
                session_id=session.id,
                selected_text=selected_text or ""
            )

            if result and hasattr(result, 'content'):
                response = result.content
            else:
                # Fallback to original implementation if agent system fails
                query_analysis = await self.query_processor.process_query(query)

                # Retrieve relevant context from the textbook
                if selected_text:
                    # If user selected specific text, use that as context
                    context_chunks = [{
                        "content": selected_text,
                        "source": "user_selected_text",
                        "section": "Selected Text",
                        "chapter": "User Selection",
                        "similarity_score": 1.0
                    }]
                else:
                    # Otherwise, retrieve relevant context from the textbook
                    context_chunks = await self._retrieve_context(session.textbook_id, query_analysis["processed_query"])

                # Generate response using the retrieved context
                response = await self._generate_response(query, context_chunks, query_analysis)

            # Add the query and response to the session
            user_message = ChatbotMessage(
                id=f"msg-{len(session.messages) + 1}-user",
                role="user",
                content=query,
                query_analysis=getattr(query_analysis, 'query', {}) if 'query_analysis' in locals() else {}
            )
            assistant_message = ChatbotMessage(
                id=f"msg-{len(session.messages) + 2}-assistant",
                role="assistant",
                content=response,
                context_sources=[]  # Will be populated from agent result if available
            )

            session.messages.extend([user_message, assistant_message])

            return response
        except Exception as e:
            self.logger.error(f"Error processing query: {str(e)}")
            return f"I encountered an error processing your query: {str(e)}. Please try again."

    async def _retrieve_context(self, textbook_id: str, query: str) -> List[Dict[str, Any]]:
        """
        Retrieve relevant context from the textbook based on the query.
        """
        try:
            # Search for relevant content chunks in the vector store
            relevant_chunks = await self.vector_store_service.search_similar_chunks(
                textbook_id=textbook_id,
                query=query,
                top_k=5
            )

            # Format the chunks for use in response generation
            formatted_chunks = []
            for chunk in relevant_chunks:
                formatted_chunks.append({
                    "content": chunk.get("content", ""),
                    "source": chunk.get("source", "unknown"),
                    "section": chunk.get("metadata", {}).get("section_title", "Unknown Section"),
                    "chapter": chunk.get("metadata", {}).get("chapter_title", "Unknown Chapter"),
                    "similarity_score": chunk.get("similarity_score", 0.0)
                })

            return formatted_chunks
        except Exception as e:
            self.logger.error(f"Error retrieving context: {str(e)}")
            # Return a default response if retrieval fails
            return [{
                "content": f"Unable to retrieve specific context for query: {query}",
                "source": "system",
                "section": "System Message",
                "chapter": "System Message",
                "similarity_score": 0.0
            }]

    async def _generate_response(self, query: str, context_chunks: List[Dict[str, Any]], query_analysis: Dict[str, Any]) -> str:
        """
        Generate a response using the query and retrieved context.
        """
        try:
            # Combine context chunks into a single context string
            context_parts = []
            for chunk in context_chunks:
                if chunk["content"].strip():
                    context_parts.append(f"Section: {chunk['section']}\nContent: {chunk['content']}")

            context_str = "\n\n".join(context_parts)

            # Create a detailed prompt for the LLM
            prompt = f"""
            You are an AI assistant for a Physical AI and Humanoid Robotics textbook.
            Answer the user's question based on the provided context from the textbook.

            Question: {query}

            Context from textbook:
            {context_str}

            Query Analysis:
            - Query type: {query_analysis.get('query_type', 'general')}
            - Intent: {query_analysis.get('intent', 'question')}
            - Key entities: {', '.join([entity['name'] for entity in query_analysis.get('entities', [])])}

            Please provide a helpful, accurate answer based on the textbook content.
            If the context doesn't contain the answer, acknowledge this and suggest where they might find the information.
            """

            # Generate response using OpenAI
            try:
                response = self.openai_client.generate_chat_response(
                    query=prompt,
                    context=context_str
                )
            except Exception as e:
                self.logger.error(f"Error with OpenAI/Google client: {str(e)}")
                # Fallback response
                response = f"Based on the textbook content, I can tell you that this is a topic about {query.split()[0] if query.split() else 'this subject'}. For more detailed information, please refer to the textbook."

            return response
        except Exception as e:
            self.logger.error(f"Error generating response: {str(e)}")
            return f"I'm sorry, I encountered an error generating a response. Please try rephrasing your question."

    async def index_textbook_content(self, textbook: Textbook) -> bool:
        """
        Index textbook content for RAG retrieval using sub-agent architecture.
        """
        try:
            # Convert textbook content to a format suitable for chunking
            textbook_data = {
                "id": textbook.id,
                "title": textbook.title,
                "chapters": []
            }

            for chapter in textbook.chapters:
                # Import section model if not already available
                from ...models.section import Section
                from typing import cast

                chapter_data = {
                    "id": chapter.id,
                    "title": chapter.title,
                    "content": chapter.content or "",
                    "sections": []
                }

                # Process sections within the chapter
                for section in chapter.sections:
                    section_data = {
                        "id": getattr(section, 'id', 'unknown'),
                        "title": getattr(section, 'title', 'Untitled Section'),
                        "content": getattr(section, 'content', '') or "",
                        "metadata": getattr(section, 'metadata', {}) or {}
                    }
                    chapter_data["sections"].append(section_data)

                textbook_data["chapters"].append(chapter_data)

            # Chunk the textbook content
            content_chunks = self.content_chunker.chunk_textbook_content(
                textbook_id=textbook.id,
                chapters=textbook_data["chapters"]
            )

            # Convert ContentChunk objects to dictionaries for storage
            chunk_dicts = []
            for chunk in content_chunks:
                chunk_dict = {
                    "id": chunk.id,
                    "content": chunk.content,
                    "source": f"{textbook.id}/{chunk.chapter}/{chunk.section}",
                    "metadata": {
                        "chapter_title": chunk.chapter,
                        "section_title": chunk.section,
                        **chunk.metadata
                    }
                }
                chunk_dicts.append(chunk_dict)

            # Use agent system to store chunks in vector store (fallback to direct call if agent unavailable)
            try:
                # Try to use the agent system
                routing_agent = self.agent_manager.get_agent("routing_agent")
                if routing_agent:
                    from ...agents.base_agent import AgentMessage

                    index_message = AgentMessage(
                        sender="rag_service",
                        recipient="retrieval_agent",
                        content="",
                        metadata={
                            "textbook_id": textbook.id,
                            "chunks": chunk_dicts
                        },
                        message_type="index_content"
                    )

                    result = await routing_agent.process_message(index_message)
                    success = result and result.metadata.get("success", False) if result else False
                else:
                    # Fallback to direct vector store call
                    success = await self.vector_store_service.add_content_chunks(
                        textbook_id=textbook.id,
                        chunks=chunk_dicts
                    )
            except Exception as agent_error:
                self.logger.warning(f"Agent-based indexing failed, using fallback: {str(agent_error)}")
                # Fallback to direct vector store call
                success = await self.vector_store_service.add_content_chunks(
                    textbook_id=textbook.id,
                    chunks=chunk_dicts
                )

            if success:
                # Update textbook to indicate it's RAG indexed
                textbook.rag_indexed = True
                textbook.rag_index_id = f"index_{textbook.id}"
                textbook.rag_last_indexed = datetime.now()

            return success
        except Exception as e:
            self.logger.error(f"Failed to index textbook content: {str(e)}")
            return False

    async def execute_skill(self, skill_id: str, **kwargs) -> Any:
        """
        Execute a specific skill through the skills manager.

        Args:
            skill_id: The ID of the skill to execute
            **kwargs: Parameters for the skill

        Returns:
            Result of the skill execution
        """
        try:
            result = await self.skills_manager.execute_skill(skill_id, **kwargs)
            return result
        except Exception as e:
            self.logger.error(f"Error executing skill {skill_id}: {str(e)}")
            return None

    async def get_available_skills(self) -> List[Dict[str, Any]]:
        """
        Get list of all available skills.

        Returns:
            List of skill information
        """
        try:
            return self.skills_manager.get_available_skills()
        except Exception as e:
            self.logger.error(f"Error getting available skills: {str(e)}")
            return []

    async def run_rag_with_skills(self, query: str, textbook_id: str, selected_text: str = "") -> str:
        """
        Run a complete RAG process using the skills system.

        Args:
            query: The user's query
            textbook_id: The ID of the textbook to search
            selected_text: Optional selected text to focus on

        Returns:
            Generated response
        """
        try:
            # Step 1: Retrieve relevant content using retrieval skill
            retrieval_result = await self.skills_manager.execute_skill(
                "text_retrieval",
                query=query,
                textbook_id=textbook_id,
                top_k=5
            )

            if not retrieval_result or not retrieval_result.success:
                # Fallback if retrieval fails
                context = ""
            else:
                # Format context from retrieved results
                context_chunks = retrieval_result.data
                context = "\n\n".join([chunk.get("content", "") for chunk in context_chunks])

            # Step 2: Generate response using generation skill
            generation_result = await self.skills_manager.execute_skill(
                "text_generation",
                query=query,
                context=context,
                selected_text=selected_text
            )

            if generation_result and generation_result.success:
                return generation_result.data
            else:
                return "I encountered an issue generating a response. Please try again."

        except Exception as e:
            self.logger.error(f"Error in RAG process with skills: {str(e)}")
            return f"I encountered an error processing your request: {str(e)}"

    async def search_context(self, textbook_id: str, query: str, top_k: int = 5) -> List[Dict[str, Any]]:
        """
        Search for relevant context in the indexed textbook content.
        """
        try:
            return await self.vector_store_service.search_similar_chunks(
                textbook_id=textbook_id,
                query=query,
                top_k=top_k
            )
        except Exception as e:
            self.logger.error(f"Error searching context: {str(e)}")

    def execute(self, *args, **kwargs):
        """
        Execute the service operation (required by BaseService).
        """
        # This method should be implemented based on specific RAG needs
        raise NotImplementedError("Execute method should be implemented by specific operations")