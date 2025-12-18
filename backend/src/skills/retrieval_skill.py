from typing import Any, Dict
from .base_skill import BaseSkill, SkillResult
from ..services.chatbot.vector_store_service import VectorStoreService


class TextRetrievalSkill(BaseSkill):
    """Skill for retrieving relevant text from the vector store"""

    def __init__(self):
        super().__init__(
            skill_id="text_retrieval",
            name="Text Retrieval",
            description="Retrieves relevant text chunks from the vector store based on query"
        )
        self.vector_store = VectorStoreService()

    async def execute(self, **kwargs) -> SkillResult:
        """Execute text retrieval"""
        try:
            # Validate required parameters
            if "query" not in kwargs or "textbook_id" not in kwargs:
                return SkillResult(
                    success=False,
                    data=None,
                    message="Missing required parameters: query and textbook_id"
                )

            query = kwargs["query"]
            textbook_id = kwargs["textbook_id"]
            top_k = kwargs.get("top_k", 5)
            filters = kwargs.get("filters", {})

            # Perform retrieval
            results = await self.vector_store.search_similar_chunks(
                textbook_id=textbook_id,
                query=query,
                top_k=top_k
            )

            return SkillResult(
                success=True,
                data=results,
                message=f"Retrieved {len(results)} relevant text chunks",
                metadata={"query": query, "textbook_id": textbook_id, "top_k": top_k}
            )
        except Exception as e:
            return SkillResult(
                success=False,
                data=None,
                message=f"Error during text retrieval: {str(e)}"
            )

    async def validate_inputs(self, **kwargs) -> bool:
        """Validate input parameters"""
        required_params = ["query", "textbook_id"]
        for param in required_params:
            if param not in kwargs:
                return False
        return True


class ContentIndexingSkill(BaseSkill):
    """Skill for indexing content into the vector store"""

    def __init__(self):
        super().__init__(
            skill_id="content_indexing",
            name="Content Indexing",
            description="Indexes content chunks into the vector store for retrieval"
        )
        self.vector_store = VectorStoreService()

    async def execute(self, **kwargs) -> SkillResult:
        """Execute content indexing"""
        try:
            # Validate required parameters
            if "textbook_id" not in kwargs or "chunks" not in kwargs:
                return SkillResult(
                    success=False,
                    data=None,
                    message="Missing required parameters: textbook_id and chunks"
                )

            textbook_id = kwargs["textbook_id"]
            chunks = kwargs["chunks"]

            # Add content to vector store
            success = await self.vector_store.add_content_chunks(
                textbook_id=textbook_id,
                chunks=chunks
            )

            if success:
                return SkillResult(
                    success=True,
                    data={"textbook_id": textbook_id, "chunks_count": len(chunks)},
                    message=f"Successfully indexed {len(chunks)} content chunks",
                    metadata={"textbook_id": textbook_id}
                )
            else:
                return SkillResult(
                    success=False,
                    data=None,
                    message="Failed to index content chunks"
                )
        except Exception as e:
            return SkillResult(
                success=False,
                data=None,
                message=f"Error during content indexing: {str(e)}"
            )

    async def validate_inputs(self, **kwargs) -> bool:
        """Validate input parameters"""
        required_params = ["textbook_id", "chunks"]
        for param in required_params:
            if param not in kwargs:
                return False
        return True