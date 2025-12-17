# AI-Native Textbook for Physical AI and Humanoid Robotics

This project implements an AI-native, interactive textbook for teaching Physical AI and Humanoid Robotics. The system allows educators to generate interactive textbook content using Google Cloud AI, customize the structure, and export in multiple formats.

## Features

- **AI Content Generation**: Generate textbook content based on subject, difficulty, and other parameters using Google Cloud APIs
- **Customizable Structure**: Customize chapter organization and content types
- **Multiple Export Formats**: Export textbooks in PDF, HTML, and Markdown formats
- **Interactive Elements**: Includes quizzes, summaries, and learning boosters
- **Progress Tracking**: Monitor content generation progress
- **Security Protection**: Input validation and sanitization to prevent XSS and SQL injection attacks
- **Caching Layer**: Optimized caching for translation and personalization services
- **Comprehensive Logging**: Detailed logging for API requests, generation events, and error tracking

## Architecture

The application follows a modular architecture with separate backend and frontend components:

### Backend (FastAPI)
- `/backend` - Contains the API server and business logic
- Models in `/backend/src/models`
- Services in `/backend/src/services`
- API endpoints in `/backend/src/api`
- Schemas in `/backend/src/schemas`

### Frontend (React/TypeScript)
- `/frontend` - Contains the user interface
- Components in `/frontend/src/components`
- API client in `/frontend/src/services`

## Getting Started

### Google Cloud Setup

Before running the application, you need to set up Google Cloud API:

1. Follow the instructions in [GOOGLE_API_SETUP.md](GOOGLE_API_SETUP.md) to create a Google Cloud project and service account
2. Download the service account key file as `google-service-account-key.json`
3. Place the key file in the project root directory

### Backend Setup

1. Navigate to the backend directory:
```bash
cd backend
```

2. Install dependencies:
```bash
pip install -r requirements.txt
```

3. Set up environment variables:
```bash
# Ensure GOOGLE_APPLICATION_CREDENTIALS points to your service account key
# This is already configured in .env file
```

4. Run the server:
```bash
uvicorn main:app --reload
```

### Frontend Setup

1. Navigate to the frontend directory:
```bash
cd frontend
```

2. Install dependencies:
```bash
npm install
```

3. Run the development server:
```bash
npm run dev
```

## API Endpoints

### Textbook Generation
- `POST /textbooks` - Create a new textbook
- `POST /textbooks/{id}/generate` - Generate content for a textbook
- `GET /textbooks/{id}` - Get textbook details
- `GET /textbooks/{id}/progress` - Get generation progress
- `GET /textbooks/{id}/preview` - Get textbook preview
- `POST /textbooks/{id}/structure` - Customize textbook structure
- `POST /textbooks/{id}/export` - Export textbook in specified format

## Project Structure

The project follows the specifications defined in `/specs/1-textbook-generation/`:
- `spec.md` - Feature specification
- `plan.md` - Implementation plan
- `tasks.md` - Implementation tasks
- Various supporting documents

## Implementation Status

All core features have been implemented according to the specification:
- ✅ Textbook generation with AI
- ✅ Customizable structure
- ✅ Multiple export formats
- ✅ Interactive elements
- ✅ Progress tracking
- ✅ Security protection with input validation
- ✅ Caching for improved performance
- ✅ Comprehensive logging system

## Security & Performance Features

### Security
The application includes comprehensive security middleware that:
- Validates all input to prevent XSS and SQL injection attacks
- Sanitizes user input before processing
- Logs security-related events for monitoring

### Caching
The application implements intelligent caching for:
- Translation results to improve response times
- Personalized content to reduce processing overhead
- Configurable TTL (time-to-live) for cache entries

### Logging
The application provides detailed logging for:
- API request/response tracking
- Generation event monitoring
- Error tracking with full context
- Performance metrics

## Contributing

This project was generated using Spec-Driven Development methodology. All changes should follow the established patterns in the `/specs` directory."# 3rd-AI-BOOK" 
