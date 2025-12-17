from typing import List, Dict, Any, Optional
import logging
import re
from dataclasses import dataclass

logger = logging.getLogger(__name__)


@dataclass
class ContentChunk:
    """
    Data class representing a content chunk for RAG
    """
    id: str
    content: str
    chapter: str
    section: str
    metadata: Dict[str, Any]
    similarity_score: Optional[float] = None


class ContentChunker:
    """
    Service for chunking textbook content into smaller pieces suitable for RAG
    """

    def __init__(self, max_chunk_size: int = 500, overlap_size: int = 50):
        """
        Initialize the content chunker

        Args:
            max_chunk_size: Maximum number of words per chunk
            overlap_size: Number of words to overlap between chunks
        """
        self.max_chunk_size = max_chunk_size
        self.overlap_size = overlap_size

    def chunk_textbook_content(
        self,
        textbook_id: str,
        chapters: List[Dict[str, Any]]
    ) -> List[ContentChunk]:
        """
        Chunk the entire textbook content
        """
        try:
            all_chunks = []

            for chapter in chapters:
                chapter_chunks = self.chunk_chapter(chapter, textbook_id)
                all_chunks.extend(chapter_chunks)

            logger.info(f"Created {len(all_chunks)} chunks for textbook {textbook_id}")
            return all_chunks

        except Exception as e:
            logger.error(f"Error chunking textbook content: {str(e)}")
            raise

    def chunk_chapter(
        self,
        chapter: Dict[str, Any],
        textbook_id: str
    ) -> List[ContentChunk]:
        """
        Chunk a single chapter's content
        """
        try:
            chunks = []
            chapter_id = chapter.get("id", "unknown")
            chapter_title = chapter.get("title", "Untitled Chapter")
            chapter_content = chapter.get("content", "")

            # Chunk the main chapter content
            main_chunks = self._chunk_text(
                chapter_content,
                f"{textbook_id}_chapter_{chapter_id}",
                chapter_title,
                "main_content"
            )

            chunks.extend(main_chunks)

            # Chunk individual sections if they exist
            sections = chapter.get("sections", [])
            for section in sections:
                section_id = section.get("id", "unknown")
                section_title = section.get("title", "Untitled Section")
                section_content = section.get("content", "")

                section_chunks = self._chunk_text(
                    section_content,
                    f"{textbook_id}_section_{section_id}",
                    chapter_title,
                    section_title
                )

                chunks.extend(section_chunks)

            return chunks

        except Exception as e:
            logger.error(f"Error chunking chapter: {str(e)}")
            raise

    def _chunk_text(
        self,
        text: str,
        base_id: str,
        chapter_title: str,
        section_title: str
    ) -> List[ContentChunk]:
        """
        Chunk a text into smaller pieces
        """
        if not text:
            return []

        # Split text into sentences to maintain context
        sentences = self._split_into_sentences(text)

        chunks = []
        current_chunk = []
        current_word_count = 0
        chunk_idx = 0

        for sentence in sentences:
            sentence_words = sentence.split()
            sentence_word_count = len(sentence_words)

            # If adding this sentence would exceed the chunk size
            if current_word_count + sentence_word_count > self.max_chunk_size:
                # If the current chunk is not empty, save it
                if current_chunk:
                    chunk_text = " ".join(current_chunk)
                    chunk_id = f"{base_id}_chunk_{chunk_idx}"

                    chunk = ContentChunk(
                        id=chunk_id,
                        content=chunk_text,
                        chapter=chapter_title,
                        section=section_title,
                        metadata={
                            "chunk_index": chunk_idx,
                            "word_count": len(chunk_text.split()),
                            "original_text_length": len(chunk_text)
                        }
                    )

                    chunks.append(chunk)
                    chunk_idx += 1

                    # Add overlap from the previous chunk
                    if self.overlap_size > 0:
                        # Get the last few words from the current chunk as overlap
                        words = chunk_text.split()
                        overlap_start = max(0, len(words) - self.overlap_size)
                        current_chunk = words[overlap_start:]
                        current_word_count = len(current_chunk)
                    else:
                        current_chunk = []
                        current_word_count = 0

                # If sentence is longer than max_chunk_size, split it
                if sentence_word_count > self.max_chunk_size:
                    sub_chunks = self._split_large_sentence(sentence)
                    for sub_chunk in sub_chunks:
                        sub_chunk_id = f"{base_id}_chunk_{chunk_idx}"
                        sub_chunk_text = " ".join(sub_chunk)

                        chunk = ContentChunk(
                            id=sub_chunk_id,
                            content=sub_chunk_text,
                            chapter=chapter_title,
                            section=section_title,
                            metadata={
                                "chunk_index": chunk_idx,
                                "word_count": len(sub_chunk_text.split()),
                                "original_text_length": len(sub_chunk_text),
                                "is_split_sentence": True
                            }
                        )

                        chunks.append(chunk)
                        chunk_idx += 1
                    continue

            # Add sentence to current chunk
            current_chunk.extend(sentence_words)
            current_word_count += sentence_word_count

        # Add the last chunk if it has content
        if current_chunk:
            chunk_text = " ".join(current_chunk)
            chunk_id = f"{base_id}_chunk_{chunk_idx}"

            chunk = ContentChunk(
                id=chunk_id,
                content=chunk_text,
                chapter=chapter_title,
                section=section_title,
                metadata={
                    "chunk_index": chunk_idx,
                    "word_count": len(chunk_text.split()),
                    "original_text_length": len(chunk_text)
                }
            )

            chunks.append(chunk)

        return chunks

    def _split_into_sentences(self, text: str) -> List[str]:
        """
        Split text into sentences using regex
        """
        # Split on sentence-ending punctuation followed by whitespace
        sentence_pattern = r'[.!?]+\s+|[\n\r]+'
        sentences = re.split(sentence_pattern, text)

        # Clean up sentences
        cleaned_sentences = []
        for sentence in sentences:
            # Remove extra whitespace
            sentence = sentence.strip()
            if sentence:
                cleaned_sentences.append(sentence)

        return cleaned_sentences

    def _split_large_sentence(self, sentence: str) -> List[List[str]]:
        """
        Split a sentence that's too large into smaller chunks
        """
        words = sentence.split()
        chunks = []

        for i in range(0, len(words), self.max_chunk_size):
            chunk = words[i:i + self.max_chunk_size]
            chunks.append(chunk)

        return chunks

    def merge_chunks(
        self,
        chunks: List[ContentChunk],
        max_merged_size: int = 1000
    ) -> List[ContentChunk]:
        """
        Merge smaller chunks together to optimize for context window
        """
        if not chunks:
            return []

        merged_chunks = []
        current_chunk_content = ""
        current_chunk_metadata = {"merged_chunks": [], "word_count": 0}
        current_chunk_id = ""
        current_chapter = ""
        current_section = ""

        for chunk in chunks:
            chunk_content = chunk.content
            chunk_word_count = len(chunk_content.split())

            # Check if adding this chunk would exceed the max size
            if (current_chunk_metadata["word_count"] + chunk_word_count) <= max_merged_size:
                # Add to current merged chunk
                if current_chunk_content:
                    current_chunk_content += " " + chunk_content
                else:
                    current_chunk_content = chunk_content

                current_chunk_metadata["merged_chunks"].append(chunk.id)
                current_chunk_metadata["word_count"] += chunk_word_count

                # Keep track of chapter and section (use the first one for the merged chunk)
                if not current_chapter:
                    current_chapter = chunk.chapter
                if not current_section:
                    current_section = chunk.section
                if not current_chunk_id:
                    current_chunk_id = f"merged_{chunk.id.split('_chunk_')[0]}"
            else:
                # Save the current merged chunk and start a new one
                if current_chunk_content:
                    merged_chunk = ContentChunk(
                        id=current_chunk_id,
                        content=current_chunk_content,
                        chapter=current_chapter,
                        section=current_section,
                        metadata=current_chunk_metadata
                    )
                    merged_chunks.append(merged_chunk)

                # Start new chunk with current content
                current_chunk_content = chunk_content
                current_chunk_metadata = {
                    "merged_chunks": [chunk.id],
                    "word_count": chunk_word_count
                }
                current_chapter = chunk.chapter
                current_section = chunk.section
                current_chunk_id = f"merged_{chunk.id.split('_chunk_')[0]}"

        # Add the last merged chunk
        if current_chunk_content:
            merged_chunk = ContentChunk(
                id=current_chunk_id,
                content=current_chunk_content,
                chapter=current_chapter,
                section=current_section,
                metadata=current_chunk_metadata
            )
            merged_chunks.append(merged_chunk)

        return merged_chunks

    def optimize_chunks_for_query(
        self,
        query: str,
        chunks: List[ContentChunk],
        top_k: int = 5
    ) -> List[ContentChunk]:
        """
        Optimize chunks for a specific query by ranking and selecting the most relevant ones
        """
        # In a real implementation, this would use semantic similarity to rank chunks
        # For now, we'll just return the first top_k chunks
        return chunks[:top_k]