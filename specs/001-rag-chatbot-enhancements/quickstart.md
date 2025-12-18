# Quickstart: RAG Chatbot with Sub-Agent Architecture

## Prerequisites

- Python 3.11+
- Node.js 18+
- Docker (for local Qdrant)
- Google Cloud account (for API access)

## Setup

### 1. Clone the Repository
```bash
git clone <repository-url>
cd <repository-name>
```

### 2. Backend Setup
```bash
cd backend
pip install -r requirements.txt
```

### 3. Frontend Setup
```bash
cd frontend
npm install
```

### 4. Environment Configuration
Create a `.env` file in the root directory with the following:

```env
# Google API Configuration
GOOGLE_APPLICATION_CREDENTIALS=google-service-account-key.json
GOOGLE_API_KEY=your_google_api_key_here

# Qdrant Vector Store Configuration
QDRANT_URL=your_qdrant_url_here
QDRANT_API_KEY=your_qdrant_api_key_here

# For local Qdrant (development)
# QDRANT_URL=http://localhost:6333

# Application Configuration
BACKEND_PORT=8000
FRONTEND_PORT=3000
```

### 5. Start Qdrant Vector Store
For local development:
```bash
docker run -p 6333:6333 -p 6334:6334 qdrant/qdrant
```

## Running the Application

### 1. Start Backend Server
```bash
cd backend
uvicorn main:app --reload --port 8000
```

### 2. Start Frontend Server
```bash
cd frontend
npm run dev
```

## Key Components

### RAG Service
The core service that manages the Retrieval-Augmented Generation pipeline:
- Content retrieval from vector store
- Context-aware response generation
- Session management

### Sub-Agent Architecture
- **Coordinator Agent**: Manages the overall RAG workflow
- **Retrieval Agent**: Handles content retrieval from vector store
- **Generation Agent**: Processes responses using LLM
- **Routing Agent**: Directs messages between agents

### Skills Framework
Modular functionality through skills:
- Text retrieval
- Content indexing
- Response generation
- Content chunking

## API Endpoints

### Chatbot Query
```
POST /chatbot/query
```
Process a query using the RAG system.

### Create Session
```
POST /chatbot/sessions
```
Create a new chatbot session.

### Index Textbook
```
POST /chatbot/index-textbook/{textbook_id}
```
Index a textbook for RAG search.

## Frontend Integration

The chatbot widget can be integrated into any page:
```typescript
import ChatbotWidget from './components/chatbot/chatbot-widget';

<ChatbotWidget textbookId="your-textbook-id" />
```

## Testing

### Backend Tests
```bash
cd backend
python -m pytest tests/test_chatbot.py -v
```

### Frontend Tests
```bash
cd frontend
npm test
```

## Troubleshooting

### Qdrant Connection Issues
- Ensure Qdrant is running and accessible
- Check QDRANT_URL and QDRANT_API_KEY in environment
- Verify Qdrant collection is created

### Google API Issues
- Verify GOOGLE_API_KEY is valid
- Check Google Cloud project has required APIs enabled
- Ensure proper authentication credentials

### Frontend Connection Issues
- Verify backend is running on specified port
- Check API_BASE_URL in frontend environment
- Ensure CORS is properly configured