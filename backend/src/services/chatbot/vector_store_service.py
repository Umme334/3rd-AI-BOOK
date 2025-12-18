from typing import List, Dict, Any, Optional
import logging
import uuid
from qdrant_client import QdrantClient
from qdrant_client.http import models
from qdrant_client.http.models import Distance, VectorParams, PointStruct
from sentence_transformers import SentenceTransformer
import numpy as np
import hashlib
import asyncio

logger = logging.getLogger(__name__)


class QdrantVectorStoreService:
    """
    Service for managing vector storage and retrieval using Qdrant
    """

    def __init__(self):
        # Initialize Qdrant client
        self.client = QdrantClient(
            host="localhost",
            port=6333
        )

        # Initialize sentence transformer for embeddings
        self.encoder = SentenceTransformer('all-MiniLM-L6-v2')

        # Collection name for textbook content
        self.collection_name = "textbook_content"

        # Create collection if it doesn't exist
        self._create_collection()

    def _create_collection(self):
        """
        Create Qdrant collection for storing textbook content
        """
        try:
            # Check if collection exists
            collections = self.client.get_collections()
            collection_names = [col.name for col in collections.collections]

            if self.collection_name not in collection_names:
                # Create collection with vector configuration
                self.client.create_collection(
                    collection_name=self.collection_name,
                    vectors_config=VectorParams(size=384, distance=Distance.COSINE)  # all-MiniLM-L6-v2 produces 384-dim vectors
                )

                logger.info(f"Created Qdrant collection: {self.collection_name}")
            else:
                logger.info(f"Qdrant collection {self.collection_name} already exists")

        except Exception as e:
            logger.error(f"Error creating Qdrant collection: {str(e)}")
            raise

    async def add_content_chunks(
        self,
        textbook_id: str,
        chunks: List[Dict[str, Any]]
    ) -> bool:
        """
        Add content chunks to the Qdrant vector store for a textbook
        """
        try:
            points = []

            for chunk in chunks:
                # Generate embeddings for the content
                content_embedding = self.encoder.encode(chunk.get("content", ""))

                # Create a unique ID for this chunk
                chunk_id = str(uuid.uuid5(
                    uuid.NAMESPACE_DNS,
                    f"{textbook_id}_{chunk.get('id', str(uuid.uuid4()))}"
                ))

                # Prepare metadata
                metadata = {
                    "textbook_id": textbook_id,
                    "chunk_id": chunk.get("id", ""),
                    "source": chunk.get("source", ""),
                    "chapter_title": chunk.get("metadata", {}).get("chapter_title", ""),
                    "section_title": chunk.get("metadata", {}).get("section_title", ""),
                    **chunk.get("metadata", {})
                }

                # Create point structure
                point = PointStruct(
                    id=chunk_id,
                    vector=content_embedding.tolist(),
                    payload=metadata
                )

                points.append(point)

            # Upload points to Qdrant
            if points:
                self.client.upsert(
                    collection_name=self.collection_name,
                    points=points
                )

                logger.info(f"Successfully added {len(points)} chunks for textbook {textbook_id}")
                return True

            return True  # No chunks to add, but operation was successful

        except Exception as e:
            logger.error(f"Error adding content chunks to Qdrant: {str(e)}")
            return False

    async def search_similar_chunks(
        self,
        textbook_id: str,
        query: str,
        top_k: int = 5
    ) -> List[Dict[str, Any]]:
        """
        Search for similar content chunks in the Qdrant vector store
        """
        try:
            # Generate embedding for the query
            query_embedding = self.encoder.encode(query).tolist()

            # Prepare filter to only search within the specific textbook
            query_filter = models.Filter(
                must=[
                    models.FieldCondition(
                        key="textbook_id",
                        match=models.MatchValue(value=textbook_id)
                    )
                ]
            )

            # Perform search in Qdrant
            search_results = self.client.search(
                collection_name=self.collection_name,
                query_vector=query_embedding,
                query_filter=query_filter,
                limit=top_k,
                with_payload=True
            )

            # Format results
            results = []
            for result in search_results:
                chunk_data = {
                    "id": result.id,
                    "content": result.payload.get("content", ""),
                    "source": result.payload.get("source", ""),
                    "chapter": result.payload.get("chapter_title", "Unknown Chapter"),
                    "section": result.payload.get("section_title", "Unknown Section"),
                    "similarity_score": result.score,
                    "metadata": result.payload
                }
                results.append(chunk_data)

            logger.info(f"Found {len(results)} similar chunks for query in textbook {textbook_id}")
            return results

        except Exception as e:
            logger.error(f"Error searching similar chunks in Qdrant: {str(e)}")
            return []

    async def delete_textbook_content(self, textbook_id: str) -> bool:
        """
        Remove all content chunks for a textbook from the Qdrant vector store
        """
        try:
            # Prepare filter to find points for this textbook
            filter_condition = models.Filter(
                must=[
                    models.FieldCondition(
                        key="textbook_id",
                        match=models.MatchValue(value=textbook_id)
                    )
                ]
            )

            # Delete points matching the filter
            self.client.delete(
                collection_name=self.collection_name,
                points_selector=models.FilterSelector(
                    filter=filter_condition
                )
            )

            logger.info(f"Deleted all chunks for textbook {textbook_id}")
            return True

        except Exception as e:
            logger.error(f"Error deleting textbook content from Qdrant: {str(e)}")
            return False

    async def update_content_chunk(
        self,
        textbook_id: str,
        chunk_id: str,
        new_content: str
    ) -> bool:
        """
        Update a specific content chunk in the Qdrant vector store
        """
        try:
            # Generate new embedding for the updated content
            new_embedding = self.encoder.encode(new_content).tolist()

            # Prepare updated payload
            updated_payload = {
                "textbook_id": textbook_id,
                "chunk_id": chunk_id,
                "content": new_content,
                "updated_at": str(uuid.uuid4())  # Add timestamp
            }

            # Update the point in Qdrant
            # Note: Qdrant doesn't have a direct update operation, so we need to upsert
            point = PointStruct(
                id=chunk_id,
                vector=new_embedding,
                payload=updated_payload
            )

            self.client.upsert(
                collection_name=self.collection_name,
                points=[point]
            )

            logger.info(f"Updated chunk {chunk_id} for textbook {textbook_id}")
            return True

        except Exception as e:
            logger.error(f"Error updating content chunk in Qdrant: {str(e)}")
            return False

    async def get_textbook_chunks_count(self, textbook_id: str) -> int:
        """
        Get the count of chunks for a specific textbook
        """
        try:
            # Prepare filter to count points for this textbook
            filter_condition = models.Filter(
                must=[
                    models.FieldCondition(
                        key="textbook_id",
                        match=models.MatchValue(value=textbook_id)
                    )
                ]
            )

            # Count points matching the filter
            count_result = self.client.count(
                collection_name=self.collection_name,
                count_filter=filter_condition
            )

            return count_result.count

        except Exception as e:
            logger.error(f"Error counting textbook chunks in Qdrant: {str(e)}")
            return 0


# Backward compatibility for existing code
VectorStoreService = QdrantVectorStoreService