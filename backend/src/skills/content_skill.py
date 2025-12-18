from typing import Any, Dict
from .base_skill import BaseSkill, SkillResult
from ..services.chatbot.content_chunker import ContentChunker


class ContentChunkingSkill(BaseSkill):
    """Skill for chunking content into smaller pieces for RAG"""

    def __init__(self):
        super().__init__(
            skill_id="content_chunking",
            name="Content Chunking",
            description="Chunks large content into smaller pieces suitable for RAG processing"
        )
        self.content_chunker = ContentChunker()

    async def execute(self, **kwargs) -> SkillResult:
        """Execute content chunking"""
        try:
            # Validate required parameters
            if "textbook_id" not in kwargs or "chapters" not in kwargs:
                return SkillResult(
                    success=False,
                    data=None,
                    message="Missing required parameters: textbook_id and chapters"
                )

            textbook_id = kwargs["textbook_id"]
            chapters = kwargs["chapters"]
            max_chunk_size = kwargs.get("max_chunk_size", 500)
            overlap_size = kwargs.get("overlap_size", 50)

            # Update chunker settings if provided
            if max_chunk_size != 500 or overlap_size != 50:
                self.content_chunker = type(self.content_chunker)(
                    max_chunk_size=max_chunk_size,
                    overlap_size=overlap_size
                )

            # Chunk the content
            content_chunks = self.content_chunker.chunk_textbook_content(
                textbook_id=textbook_id,
                chapters=chapters
            )

            chunk_dicts = []
            for chunk in content_chunks:
                chunk_dict = {
                    "id": chunk.id,
                    "content": chunk.content,
                    "source": f"{textbook_id}/{chunk.chapter}/{chunk.section}",
                    "metadata": {
                        "chapter_title": chunk.chapter,
                        "section_title": chunk.section,
                        **chunk.metadata
                    }
                }
                chunk_dicts.append(chunk_dict)

            return SkillResult(
                success=True,
                data=chunk_dicts,
                message=f"Successfully chunked content into {len(chunk_dicts)} pieces",
                metadata={
                    "textbook_id": textbook_id,
                    "total_chunks": len(chunk_dicts),
                    "max_chunk_size": max_chunk_size,
                    "overlap_size": overlap_size
                }
            )
        except Exception as e:
            return SkillResult(
                success=False,
                data=None,
                message=f"Error during content chunking: {str(e)}"
            )

    async def validate_inputs(self, **kwargs) -> bool:
        """Validate input parameters"""
        required_params = ["textbook_id", "chapters"]
        for param in required_params:
            if param not in kwargs:
                return False
        return True


class ContentAnalysisSkill(BaseSkill):
    """Skill for analyzing content characteristics"""

    def __init__(self):
        super().__init__(
            skill_id="content_analysis",
            name="Content Analysis",
            description="Analyzes content for characteristics like readability, complexity, and key topics"
        )

    async def execute(self, **kwargs) -> SkillResult:
        """Execute content analysis"""
        try:
            # Validate required parameters
            if "content" not in kwargs:
                return SkillResult(
                    success=False,
                    data=None,
                    message="Missing required parameter: content"
                )

            content = kwargs["content"]
            analysis_type = kwargs.get("analysis_type", "comprehensive")  # comprehensive, readability, keywords

            result_data = {}

            if analysis_type in ["comprehensive", "readability"]:
                # Basic readability metrics
                word_count = len(content.split())
                sentence_count = len([s for s in content.split('.') if s.strip()])
                avg_sentence_length = word_count / sentence_count if sentence_count > 0 else 0

                result_data["readability"] = {
                    "word_count": word_count,
                    "sentence_count": sentence_count,
                    "avg_sentence_length": avg_sentence_length
                }

            if analysis_type in ["comprehensive", "keywords"]:
                # Basic keyword extraction (in a real implementation, this would use NLP techniques)
                words = content.lower().split()
                common_words = ["the", "a", "an", "and", "or", "but", "in", "on", "at", "to", "for", "of", "with", "by"]
                filtered_words = [word.strip(".,!?;:\"()[]{}") for word in words if word.lower() not in common_words and len(word) > 3]

                # Get unique words with frequency
                word_freq = {}
                for word in filtered_words:
                    word_freq[word] = word_freq.get(word, 0) + 1

                # Get top 10 most frequent words
                sorted_words = sorted(word_freq.items(), key=lambda x: x[1], reverse=True)
                top_keywords = [word for word, freq in sorted_words[:10]]

                result_data["keywords"] = top_keywords

            return SkillResult(
                success=True,
                data=result_data,
                message="Successfully analyzed content",
                metadata={"analysis_type": analysis_type, "content_length": len(content)}
            )
        except Exception as e:
            return SkillResult(
                success=False,
                data=None,
                message=f"Error during content analysis: {str(e)}"
            )

    async def validate_inputs(self, **kwargs) -> bool:
        """Validate input parameters"""
        required_params = ["content"]
        for param in required_params:
            if param not in kwargs:
                return False
        return True


class TextExtractionSkill(BaseSkill):
    """Skill for extracting specific information from text"""

    def __init__(self):
        super().__init__(
            skill_id="text_extraction",
            name="Text Extraction",
            description="Extracts specific types of information from text content"
        )

    async def execute(self, **kwargs) -> SkillResult:
        """Execute text extraction"""
        try:
            # Validate required parameters
            if "content" not in kwargs or "extraction_type" not in kwargs:
                return SkillResult(
                    success=False,
                    data=None,
                    message="Missing required parameters: content and extraction_type"
                )

            content = kwargs["content"]
            extraction_type = kwargs["extraction_type"]  # entities, concepts, definitions, examples

            result_data = {}

            if extraction_type == "definitions":
                # Look for definition patterns like "X is defined as..." or "X means..."
                import re
                definition_patterns = [
                    r'([A-Za-z\s]+)(?:\s+)?(?:is defined as|means|refers to|is)(?:\s+)?([^\.]+)',
                    r'([A-Za-z\s]+)(?:\s+)?(:|=)(?:\s+)?([^\.]+)'
                ]

                definitions = []
                for pattern in definition_patterns:
                    matches = re.findall(pattern, content, re.IGNORECASE)
                    for match in matches:
                        term = match[0].strip()
                        definition = match[2].strip() if len(match) > 2 else match[1].strip()
                        if term and definition:
                            definitions.append({"term": term, "definition": definition})

                result_data["definitions"] = definitions

            elif extraction_type == "examples":
                # Look for example patterns like "For example", "e.g.", "such as"
                import re
                example_patterns = [
                    r'For example,?\s+([^\.]+)',
                    r'For instance,?\s+([^\.]+)',
                    r'e\.g\.,?\s+([^\.]+)',
                    r'such as\s+([^\.]+)'
                ]

                examples = []
                for pattern in example_patterns:
                    matches = re.findall(pattern, content, re.IGNORECASE)
                    examples.extend([match.strip() for match in matches if match.strip()])

                result_data["examples"] = examples

            elif extraction_type == "entities":
                # Basic entity extraction (in a real implementation, this would use NER)
                import re
                # Look for capitalized words that might be entities
                potential_entities = re.findall(r'\b[A-Z][a-z]+\b', content)
                # Filter for potentially meaningful entities (longer than 2 characters)
                entities = [entity for entity in set(potential_entities) if len(entity) > 2]
                result_data["entities"] = entities

            elif extraction_type == "concepts":
                # Look for technical terms and concepts (words in title case or with specific patterns)
                import re
                concept_patterns = [
                    r'\b[A-Z][a-z]+[A-Z][a-z]+\b',  # CamelCase
                    r'\b(?:[A-Z]{2,}|\b[A-Z][a-z]+\s+[A-Z][a-z]+)\b'  # Acronyms or multi-word terms
                ]

                concepts = []
                for pattern in concept_patterns:
                    matches = re.findall(pattern, content)
                    concepts.extend(matches)

                result_data["concepts"] = list(set(concepts))  # Remove duplicates

            return SkillResult(
                success=True,
                data=result_data,
                message=f"Successfully extracted {extraction_type}",
                metadata={"extraction_type": extraction_type, "content_length": len(content)}
            )
        except Exception as e:
            return SkillResult(
                success=False,
                data=None,
                message=f"Error during text extraction: {str(e)}"
            )

    async def validate_inputs(self, **kwargs) -> bool:
        """Validate input parameters"""
        required_params = ["content", "extraction_type"]
        for param in required_params:
            if param not in kwargs:
                return False
        return True