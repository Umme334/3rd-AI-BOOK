from typing import List, Dict, Any, Optional
import logging
import uuid
import os
import hashlib
import asyncio
import json

# Import sentence_transformers if available, otherwise use a simple fallback
try:
    from sentence_transformers import SentenceTransformer
    EMBEDDING_AVAILABLE = True
except ImportError:
    print("Sentence transformers not available, using fallback embedding")
    EMBEDDING_AVAILABLE = False

# Try to import Qdrant, but handle gracefully if not available
try:
    from qdrant_client import QdrantClient
    from qdrant_client.http import models
    from qdrant_client.http.models import Distance, VectorParams, PointStruct
    QDRANT_AVAILABLE = True
except ImportError:
    print("Qdrant client not available, using in-memory fallback")
    QDRANT_AVAILABLE = False

logger = logging.getLogger(__name__)


class VectorStoreService:
    """
    Service for managing vector storage and retrieval using Qdrant with fallback to in-memory storage
    """

    def __init__(self):
        if QDRANT_AVAILABLE:
            # Use Qdrant if available
            self.use_qdrant = True
            # Get Qdrant Cloud configuration from environment
            qdrant_url = os.getenv("QDRANT_URL")
            qdrant_api_key = os.getenv("QDRANT_API_KEY")

            # Fallback to local Qdrant for development unless Qdrant Cloud is properly configured
            if qdrant_url and qdrant_api_key:
                # Initialize Qdrant client for Cloud
                try:
                    self.client = QdrantClient(
                        url=qdrant_url,
                        api_key=qdrant_api_key
                    )
                    logger.info("Connected to Qdrant Cloud")
                except Exception as e:
                    logger.warning(f"Failed to connect to Qdrant Cloud: {str(e)}, falling back to local instance")
                    self.client = QdrantClient(
                        host="localhost",
                        port=6333
                    )
            else:
                # Fallback to local Qdrant for development
                self.client = QdrantClient(
                    host="localhost",
                    port=6333
                )
                logger.info("Connected to local Qdrant instance")

            # Collection name for textbook content
            self.collection_name = "textbook_content"

            # Create collection if it doesn't exist
            try:
                self._create_collection()
            except Exception as e:
                logger.warning(f"Failed to create collection initially: {str(e)}, will retry when needed")
        else:
            # Use in-memory fallback
            self.use_qdrant = False
            self._in_memory_store = {}  # Dictionary to store content chunks
            logger.info("Using in-memory vector store fallback")

        # Initialize sentence transformer for embeddings if available
        if EMBEDDING_AVAILABLE:
            try:
                self.encoder = SentenceTransformer('all-MiniLM-L6-v2')
            except Exception as e:
                logger.warning(f"Failed to load sentence transformer: {str(e)}, using fallback")
                self.encoder = None
                # Don't modify the global variable here
        else:
            # Use a simple fallback encoder (this is a very basic implementation)
            self.encoder = None
            logger.warning("Using fallback embedding method - performance will be limited")

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
        Add content chunks to the vector store for a textbook
        """
        try:
            if self.use_qdrant and QDRANT_AVAILABLE:
                # Use Qdrant implementation
                points = []

                for chunk in chunks:
                    # Generate embeddings for the content
                    if EMBEDDING_AVAILABLE and self.encoder:
                        content_embedding = self.encoder.encode(chunk.get("content", ""))
                        vector = content_embedding.tolist()
                    else:
                        # Fallback: use a simple hash-based embedding (very basic)
                        content = chunk.get("content", "")
                        # Create a simple vector based on content hash
                        vector = self._simple_embedding(content)

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
                        vector=vector,
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
            else:
                # Use in-memory implementation
                if textbook_id not in self._in_memory_store:
                    self._in_memory_store[textbook_id] = []

                for chunk in chunks:
                    # Generate embeddings for the content
                    if EMBEDDING_AVAILABLE and self.encoder:
                        content_embedding = self.encoder.encode(chunk.get("content", ""))
                        vector = content_embedding.tolist()
                    else:
                        # Fallback: use a simple hash-based embedding (very basic)
                        content = chunk.get("content", "")
                        # Create a simple vector based on content hash
                        vector = self._simple_embedding(content)

                    # Create a unique ID for this chunk
                    chunk_id = str(uuid.uuid5(
                        uuid.NAMESPACE_DNS,
                        f"{textbook_id}_{chunk.get('id', str(uuid.uuid4()))}"
                    ))

                    # Prepare chunk data
                    chunk_data = {
                        "id": chunk_id,
                        "vector": vector,
                        "textbook_id": textbook_id,
                        "chunk_id": chunk.get("id", ""),
                        "source": chunk.get("source", ""),
                        "content": chunk.get("content", ""),
                        "chapter_title": chunk.get("metadata", {}).get("chapter_title", ""),
                        "section_title": chunk.get("metadata", {}).get("section_title", ""),
                        **chunk.get("metadata", {})
                    }

                    self._in_memory_store[textbook_id].append(chunk_data)

                logger.info(f"Successfully added {len(chunks)} chunks for textbook {textbook_id} to in-memory store")
                return True

        except Exception as e:
            logger.error(f"Error adding content chunks to vector store: {str(e)}")
            return False

    async def search_similar_chunks(
        self,
        textbook_id: str,
        query: str,
        top_k: int = 5
    ) -> List[Dict[str, Any]]:
        """
        Search for similar content chunks in the vector store
        """
        try:
            if self.use_qdrant and QDRANT_AVAILABLE:
                # Use Qdrant implementation
                if EMBEDDING_AVAILABLE and self.encoder:
                    # Generate embedding for the query using sentence transformers
                    query_embedding = self.encoder.encode(query).tolist()
                else:
                    # Fallback: use simple embedding for query
                    query_embedding = self._simple_embedding(query)

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
            else:
                # Use in-memory implementation with cosine similarity
                if EMBEDDING_AVAILABLE and self.encoder:
                    # Generate embedding for the query using sentence transformers
                    query_embedding = self.encoder.encode(query).tolist()
                else:
                    # Fallback: use simple embedding for query
                    query_embedding = self._simple_embedding(query)

                # Get chunks for the textbook
                textbook_chunks = self._in_memory_store.get(textbook_id, [])

                # Calculate similarity scores using cosine similarity
                scored_chunks = []
                for chunk in textbook_chunks:
                    similarity = self._cosine_similarity(query_embedding, chunk.get("vector", []))
                    scored_chunks.append((chunk, similarity))

                # Sort by similarity score (descending) and return top_k
                scored_chunks.sort(key=lambda x: x[1], reverse=True)
                top_chunks = scored_chunks[:top_k]

                # Format results
                results = []
                for chunk, similarity in top_chunks:
                    chunk_data = {
                        "id": chunk.get("id", ""),
                        "content": chunk.get("content", ""),
                        "source": chunk.get("source", ""),
                        "chapter": chunk.get("chapter_title", "Unknown Chapter"),
                        "section": chunk.get("section_title", "Unknown Section"),
                        "similarity_score": similarity,
                        "metadata": chunk
                    }
                    results.append(chunk_data)

                logger.info(f"Found {len(results)} similar chunks for query in textbook {textbook_id}")
                return results

        except Exception as e:
            logger.error(f"Error searching similar chunks in vector store: {str(e)}")
            return []

    def _cosine_similarity(self, vec1: List[float], vec2: List[float]) -> float:
        """
        Calculate cosine similarity between two vectors
        """
        try:
            # Calculate dot product
            dot_product = sum(a * b for a, b in zip(vec1, vec2))

            # Calculate magnitudes
            magnitude1 = sum(a * a for a in vec1) ** 0.5
            magnitude2 = sum(b * b for b in vec2) ** 0.5

            # Calculate cosine similarity
            if magnitude1 == 0 or magnitude2 == 0:
                return 0.0

            return dot_product / (magnitude1 * magnitude2)
        except:
            # Fallback similarity calculation
            return 0.0

    def _simple_embedding(self, text: str, vector_size: int = 384) -> List[float]:
        """
        Create a simple embedding using text hashing - fallback when sentence_transformers is not available
        """
        import hashlib
        import math

        # Create a hash of the text
        text_hash = hashlib.md5(text.encode()).hexdigest()

        # Convert hash to a sequence of numbers and normalize
        vector = []
        for i in range(vector_size):
            # Use different parts of the hash to create the vector
            start_idx = (i * 2) % len(text_hash)
            end_idx = ((i * 2) + 4) % len(text_hash)
            if start_idx > end_idx:  # Handle wrap-around
                hash_part = text_hash[start_idx:] + text_hash[:end_idx]
            else:
                hash_part = text_hash[start_idx:end_idx]

            # Convert hex to float and normalize
            value = int(hash_part, 16) / (16 ** len(hash_part))  # Normalize to 0-1
            value = (value * 2) - 1  # Scale to -1 to 1 range
            vector.append(value)

        return vector

    async def delete_textbook_content(self, textbook_id: str) -> bool:
        """
        Remove all content chunks for a textbook from the vector store
        """
        try:
            if self.use_qdrant and QDRANT_AVAILABLE:
                # Use Qdrant implementation
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
            else:
                # Use in-memory implementation
                if textbook_id in self._in_memory_store:
                    del self._in_memory_store[textbook_id]
                    logger.info(f"Deleted all chunks for textbook {textbook_id} from in-memory store")
                    return True
                else:
                    logger.info(f"No chunks found for textbook {textbook_id} in in-memory store")
                    return True  # Operation successful even if no chunks existed

        except Exception as e:
            logger.error(f"Error deleting textbook content from vector store: {str(e)}")
            return False

    async def update_content_chunk(
        self,
        textbook_id: str,
        chunk_id: str,
        new_content: str
    ) -> bool:
        """
        Update a specific content chunk in the vector store
        """
        try:
            if self.use_qdrant and QDRANT_AVAILABLE:
                # Use Qdrant implementation
                if EMBEDDING_AVAILABLE and self.encoder:
                    # Generate new embedding for the updated content
                    new_embedding = self.encoder.encode(new_content).tolist()
                else:
                    # Fallback: use simple embedding
                    new_embedding = self._simple_embedding(new_content)

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
            else:
                # Use in-memory implementation
                if textbook_id in self._in_memory_store:
                    # Find the chunk with the given ID
                    for i, chunk in enumerate(self._in_memory_store[textbook_id]):
                        if chunk.get("id") == chunk_id:
                            # Update the content
                            if EMBEDDING_AVAILABLE and self.encoder:
                                # Generate new embedding for the updated content
                                new_embedding = self.encoder.encode(new_content).tolist()
                            else:
                                # Fallback: use simple embedding
                                new_embedding = self._simple_embedding(new_content)

                            # Update the chunk
                            self._in_memory_store[textbook_id][i]["content"] = new_content
                            self._in_memory_store[textbook_id][i]["vector"] = new_embedding
                            self._in_memory_store[textbook_id][i]["updated_at"] = str(uuid.uuid4())

                            logger.info(f"Updated chunk {chunk_id} for textbook {textbook_id} in in-memory store")
                            return True

                logger.warning(f"Chunk {chunk_id} not found in textbook {textbook_id}")
                return False

        except Exception as e:
            logger.error(f"Error updating content chunk in vector store: {str(e)}")
            return False

    async def get_textbook_chunks_count(self, textbook_id: str) -> int:
        """
        Get the count of chunks for a specific textbook
        """
        try:
            if self.use_qdrant and QDRANT_AVAILABLE:
                # Use Qdrant implementation
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
            else:
                # Use in-memory implementation
                return len(self._in_memory_store.get(textbook_id, []))

        except Exception as e:
            logger.error(f"Error counting textbook chunks in vector store: {str(e)}")
            return 0


