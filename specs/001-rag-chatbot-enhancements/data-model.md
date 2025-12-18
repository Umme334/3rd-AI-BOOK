# Data Model: RAG Chatbot with Sub-Agent Architecture

## 1. Core Entities

### ChatbotSession
- **id**: string (UUID)
- **textbook_id**: string
- **user_id**: string (optional)
- **messages**: List<ChatbotMessage>
- **context_chunks**: List<any> (optional)
- **is_active**: boolean
- **created_at**: datetime
- **updated_at**: datetime

### ChatbotMessage
- **id**: string (UUID)
- **role**: string (user|assistant)
- **content**: string
- **timestamp**: datetime
- **query_analysis**: dict (optional)
- **context_sources**: List<string> (optional)

### Textbook
- **id**: string (UUID)
- **title**: string
- **subject**: string
- **difficulty**: string
- **chapters**: List<Chapter>
- **rag_indexed**: boolean
- **rag_index_id**: string (optional)
- **rag_last_indexed**: datetime (optional)

### Chapter
- **id**: string (UUID)
- **title**: string
- **content**: string (optional)
- **sections**: List<Section>

### Section
- **id**: string (UUID)
- **title**: string
- **content**: string
- **metadata**: dict (optional)

## 2. Content Chunking Model

### ContentChunk
- **id**: string (UUID)
- **content**: string
- **source**: string (textbook_id/chapter_id/section_id)
- **chapter**: string
- **section**: string
- **metadata**: dict
  - **chapter_title**: string
  - **section_title**: string
  - **page_number**: number (optional)
  - **vector_id**: string (Qdrant point ID)

## 3. API Request/Response Models

### ChatbotQueryRequest
- **textbook_id**: string
- **query**: string
- **session_id**: string (optional)
- **selected_text**: string (optional)

### ChatbotResponse
- **response**: string
- **session_id**: string
- **sources**: List<Record<string, any>>
- **context_sources**: List<string>
- **followup_questions**: List<string>
- **query_time**: string (ISO datetime)
- **confidence**: number
- **tokens_used**: number (optional)

### ChatbotSessionCreateRequest
- **textbook_id**: string
- **user_id**: string (optional)
- **session_name**: string (optional)

### ChatbotSessionResponse
- **session_id**: string
- **textbook_id**: string
- **user_id**: string (optional)
- **session_name**: string
- **created_at**: string (ISO datetime)
- **last_accessed**: string (ISO datetime)
- **message_count**: number
- **active**: boolean

## 4. Agent Communication Model

### AgentMessage
- **sender**: string
- **recipient**: string
- **content**: string
- **metadata**: dict
- **message_type**: string (info|query|response|index_content)
- **timestamp**: datetime

## 5. Skill Execution Model

### SkillResult
- **success**: boolean
- **data**: any (optional)
- **message**: string (optional)

## 6. Vector Store Schema

### Qdrant Collection: "textbook_content"
- **Point ID**: UUID-based
- **Vector**: 384-dimensional (all-MiniLM-L6-v2 embedding)
- **Payload**:
  - **textbook_id**: string
  - **chunk_id**: string
  - **content**: string
  - **source**: string
  - **chapter_title**: string
  - **section_title**: string
  - **metadata**: dict (additional metadata)

## 7. Relationships

- **ChatbotSession** 1 → * **ChatbotMessage** (session contains multiple messages)
- **Textbook** 1 → * **Chapter** (textbook contains multiple chapters)
- **Chapter** 1 → * **Section** (chapter contains multiple sections)
- **ContentChunk** * → 1 **Textbook** (chunks belong to a textbook)
- **ContentChunk** * → 1 **Chapter** (chunks belong to a chapter)
- **ContentChunk** * → 1 **Section** (chunks belong to a section)

## 8. Indexing Strategy

### Vector Store Indexes:
- **textbook_id**: For filtering content by textbook
- **chapter_title**: For chapter-level retrieval
- **section_title**: For section-level retrieval

### Performance Considerations:
- Chunk size: 200-400 tokens for optimal retrieval
- Overlap: 20-50 tokens to maintain context
- Metadata: Include chapter/section info for attribution