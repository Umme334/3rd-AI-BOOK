# Quickstart: Textbook Generation

## Overview
This guide will help you get started with the textbook generation feature. The system allows educators to generate interactive textbooks using AI, customize their structure, and export in multiple formats.

## Prerequisites
- Python 3.11+ installed
- Node.js 18+ installed
- OpenAI API key (for content generation)
- Basic understanding of REST APIs

## Setup

### Backend Setup
1. Install dependencies:
```bash
pip install fastapi uvicorn python-multipart openai pydantic
```

2. Set environment variables:
```bash
export OPENAI_API_KEY=your_openai_api_key_here
export API_BASE_URL=http://localhost:8000
```

3. Start the backend server:
```bash
uvicorn main:app --reload --port 8000
```

### Frontend Setup
1. Install dependencies:
```bash
npm install react react-dom axios
```

2. Start the development server:
```bash
npm run dev
```

## Basic Usage

### 1. Create a New Textbook
```bash
curl -X POST http://localhost:8000/textbooks \
  -H "Content-Type: application/json" \
  -d '{
    "title": "Introduction to Physical AI",
    "subject": "Robotics",
    "difficulty": "intermediate",
    "target_audience": "Undergraduate students",
    "chapter_count": 5
  }'
```

### 2. Start Content Generation
```bash
curl -X POST http://localhost:8000/textbooks/{textbook_id}/generate
```

### 3. Check Generation Progress
```bash
curl -X GET http://localhost:8000/textbooks/{textbook_id}/progress
```

### 4. Preview the Generated Content
```bash
curl -X GET http://localhost:8000/textbooks/{textbook_id}/preview
```

### 5. Export the Textbook
```bash
curl -X POST http://localhost:8000/textbooks/{textbook_id}/export \
  -H "Content-Type: application/json" \
  -d '{
    "format": "pdf"
  }'
```

## Key Configuration Options

- **Difficulty Levels**: beginner, intermediate, advanced
- **Export Formats**: pdf, html, markdown (Docusaurus compatible)
- **Interactive Elements**: quizzes, summaries, learning boosters
- **Chapter Count**: 1-20 chapters per textbook

## Next Steps

1. Customize the UI to match your branding
2. Add authentication for user management
3. Implement additional export formats
4. Add content validation and quality checks
5. Integrate with your existing learning management system

## Security & Performance Features

The application includes several security and performance features:

### Security
- Input validation and sanitization to prevent XSS and SQL injection
- Content filtering for user-generated content
- Secure API endpoint protection

### Performance
- Caching layer for translation and personalization services
- Optimized database queries
- Asynchronous processing for content generation

### Logging
- Detailed API request logging
- Generation event tracking
- Error logging with full context
- Performance metrics collection

## Troubleshooting

- If content generation fails, check your OpenAI API key and quota
- For export issues, ensure Pandoc is installed if using PDF export
- For performance issues, consider implementing caching for generated content