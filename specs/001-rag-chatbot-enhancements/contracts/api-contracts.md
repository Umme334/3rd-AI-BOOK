# API Contracts: RAG Chatbot with Sub-Agent Architecture

## 1. Chatbot Query Endpoint

### POST `/chatbot/query`

**Request**:
```json
{
  "textbook_id": "string",
  "query": "string",
  "session_id": "string (optional)",
  "selected_text": "string (optional)"
}
```

**Response**:
```json
{
  "response": "string",
  "session_id": "string",
  "sources": "array<object>",
  "context_sources": "array<string>",
  "followup_questions": "array<string>",
  "query_time": "string (ISO datetime)",
  "confidence": "number",
  "tokens_used": "number (optional)"
}
```

**Success**: 200 OK
**Validation Error**: 422 Unprocessable Entity
**Not Found**: 404 Not Found
**Server Error**: 500 Internal Server Error

## 2. Session Management

### POST `/chatbot/sessions`

**Request**:
```json
{
  "textbook_id": "string",
  "user_id": "string (optional)",
  "session_name": "string (optional)"
}
```

**Response**:
```json
{
  "session_id": "string",
  "textbook_id": "string",
  "user_id": "string (optional)",
  "session_name": "string",
  "created_at": "string (ISO datetime)",
  "last_accessed": "string (ISO datetime)",
  "message_count": "number",
  "active": "boolean"
}
```

### GET `/chatbot/sessions/{session_id}`

**Response**:
```json
{
  "session_id": "string",
  "textbook_id": "string",
  "user_id": "string (optional)",
  "session_name": "string",
  "created_at": "string (ISO datetime)",
  "last_accessed": "string (ISO datetime)",
  "message_count": "number",
  "active": "boolean"
}
```

### GET `/chatbot/sessions/{session_id}/history`

**Response**:
```json
{
  "session_id": "string",
  "messages": [
    {
      "id": "string",
      "role": "string (user|assistant)",
      "content": "string",
      "timestamp": "string (ISO datetime)"
    }
  ],
  "total_messages": "number"
}
```

## 3. Textbook Indexing

### POST `/chatbot/index-textbook/{textbook_id}`

**Response**:
```json
{
  "message": "string",
  "textbook_id": "string",
  "indexed_at": "string (ISO datetime)",
  "indexed_chapters_count": "number"
}
```

**Success**: 200 OK
**Not Found**: 404 Not Found
**Server Error**: 500 Internal Server Error

## 4. Session Deletion

### DELETE `/chatbot/sessions/{session_id}`

**Response**:
```json
{
  "message": "string",
  "session_id": "string"
}
```

## 5. Agent Communication Contracts

### Agent Message Structure
```json
{
  "sender": "string",
  "recipient": "string",
  "content": "string",
  "metadata": "object",
  "message_type": "string",
  "timestamp": "string (ISO datetime)"
}
```

## 6. Skills Framework Contracts

### Skill Execution Request
```json
{
  "skill_id": "string",
  "kwargs": "object"
}
```

### Skill Execution Response
```json
{
  "success": "boolean",
  "data": "any (optional)",
  "message": "string (optional)"
}
```

## 7. Error Response Format

All error responses follow this format:
```json
{
  "detail": "string"
}
```

## 8. Frontend-Backend Communication

### WebSocket (if implemented)
- Connection: `ws://localhost:8000/chatbot/ws/{session_id}`
- Message format: JSON with type and payload
- Heartbeat: Every 30 seconds to maintain connection

## 9. Authentication (if required)

Some endpoints may require authentication headers:
```
Authorization: Bearer {token}
```

## 10. Rate Limiting

- Queries per minute: 60 requests per IP
- Concurrent sessions: 10 per user
- Textbook indexing: 1 per textbook per 5 minutes