from typing import Dict, Any, List, Optional
from ..services.chatbot.vector_store_service import VectorStoreService
from .base_agent import BaseAgent, AgentMessage


class RetrievalAgent(BaseAgent):
    """Agent responsible for retrieving relevant content from vector store"""

    def __init__(self, agent_id: str, name: str):
        super().__init__(agent_id, name)
        self.vector_store = VectorStoreService()

    async def process_message(self, message: AgentMessage) -> Optional[AgentMessage]:
        """Process retrieval requests"""
        if message.message_type == "retrieve_content":
            return await self._handle_retrieve_content(message)
        elif message.message_type == "index_content":
            return await self._handle_index_content(message)
        else:
            return await self._handle_default(message)

    async def _handle_retrieve_content(self, message: AgentMessage) -> Optional[AgentMessage]:
        """Handle content retrieval requests"""
        try:
            textbook_id = message.metadata.get("textbook_id")
            query = message.content
            top_k = message.metadata.get("top_k", 5)

            # Search for similar content in vector store
            results = await self.vector_store.search_similar_chunks(
                textbook_id=textbook_id,
                query=query,
                top_k=top_k
            )

            return AgentMessage(
                sender=self.agent_id,
                recipient=message.sender,
                content="Content retrieval successful",
                metadata={"results": results, "query": query},
                message_type="retrieval_results"
            )
        except Exception as e:
            return AgentMessage(
                sender=self.agent_id,
                recipient=message.sender,
                content=f"Error during retrieval: {str(e)}",
                metadata={"error": str(e)},
                message_type="error"
            )

    async def _handle_index_content(self, message: AgentMessage) -> Optional[AgentMessage]:
        """Handle content indexing requests"""
        try:
            textbook_id = message.metadata.get("textbook_id")
            chunks = message.metadata.get("chunks", [])

            # Add content chunks to vector store
            success = await self.vector_store.add_content_chunks(
                textbook_id=textbook_id,
                chunks=chunks
            )

            return AgentMessage(
                sender=self.agent_id,
                recipient=message.sender,
                content="Content indexing completed",
                metadata={"success": success, "textbook_id": textbook_id},
                message_type="indexing_results"
            )
        except Exception as e:
            return AgentMessage(
                sender=self.agent_id,
                recipient=message.sender,
                content=f"Error during indexing: {str(e)}",
                metadata={"error": str(e)},
                message_type="error"
            )

    async def _handle_default(self, message: AgentMessage) -> Optional[AgentMessage]:
        """Handle default message"""
        return AgentMessage(
            sender=self.agent_id,
            recipient=message.sender,
            content="Retrieval agent received message",
            metadata={"handled_by": self.name},
            message_type="acknowledged"
        )