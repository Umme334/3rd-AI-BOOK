# Implementation Plan: AI-Native Textbook for Physical AI and Humanoid Robotics

**Branch**: `1-textbook-generation` | **Date**: 2025-12-10 | **Spec**: [specs/1-textbook-generation/spec.md]
**Input**: Feature specification from `/specs/1-textbook-generation/spec.md`

## Summary

AI-Native textbook generation system specifically for Physical AI and Humanoid Robotics course that allows educators to generate interactive textbook content using AI. The system will accept user parameters for Physical AI topics (subject, difficulty, length) and produce structured educational content with chapters covering ROS 2, Gazebo, NVIDIA Isaac, and Vision-Language-Action modules. The system includes an integrated RAG chatbot using OpenAI Agents/ChatKit SDKs, FastAPI, Neon Serverless Postgres, and Qdrant Cloud Free Tier that answers questions based only on selected text from the book. It also features user authentication with better-auth.com, content personalization based on user's software and hardware background, and Urdu translation capabilities. Implementation will follow modular design principles to ensure scalability and maintainability for complex robotics education.

## Technical Context

**Language/Version**: Python 3.11, JavaScript/TypeScript for frontend components
**Primary Dependencies**:
- FastAPI for backend API
- Docusaurus for documentation generation and textbook publishing
- OpenAI API for content generation
- OpenAI Agents/ChatKit SDKs for RAG chatbot functionality
- Neon Serverless Postgres for user data and chatbot storage
- Qdrant Cloud Free Tier for vector storage and retrieval for the RAG system
- Better-auth.com for user authentication and authorization
- Pandoc for format conversion
**Storage**: File-based storage for generated content, PostgreSQL (Neon) for user profiles, chat history, and metadata
**Testing**: pytest for backend, Jest for frontend components
**Target Platform**: Web application deployable on GitHub Pages via Docusaurus
**Project Type**: Web application with backend API for content generation and RAG chatbot services
**Performance Goals**: Generate basic textbook (3-5 chapters) in under 5 minutes; RAG chatbot responds to queries in under 5 seconds
**Constraints**: Must work on free tiers (Qdrant + Neon), mobile-friendly UI, minimal dependencies as per constitution

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

- ✅ Modularity First: Each component (content generation, RAG chatbot, auth, personalization, translation) will be independently testable with clear interfaces
- ✅ Scalability Design: Architecture will support growth to additional subjects and textbook types beyond Physical AI
- ✅ Accessibility & Performance: Web interface will be mobile-friendly and work on limited devices; optimized for GitHub Pages deployment
- ✅ Interactive Learning Focus: Generated textbooks will include RAG chatbot, interactive elements like quizzes and summaries, personalization, and Urdu translation
- ✅ Minimal Dependencies: Will prefer lightweight solutions over complex frameworks while supporting required features
- ✅ Cloud + Edge Hybrid: Content generation and RAG services in cloud, with deployment options for Jetson kits as specified in course requirements

## Post-Design Constitution Check

*Re-evaluated after Phase 1 design completion*

- ✅ Modularity First: API contracts clearly define interfaces between components; services are independently deployable
- ✅ Scalability Design: Data model supports multiple textbook types and subjects; API design allows for future expansion of Physical AI content
- ✅ Accessibility & Performance: Frontend design prioritizes mobile responsiveness and performance; API optimized for quick responses for complex robotics topics
- ✅ Interactive Learning Focus: Data model includes dedicated entities for RAG chatbot, user profiles, personalization, and translation; API supports real-time interactions
- ✅ Minimal Dependencies: Selected technology stack keeps dependencies lightweight while supporting required features; API-first approach enables flexible frontends
- ✅ Cloud + Edge Hybrid: Content generation and RAG services in cloud with export options for offline use on Jetson kits; API supports both online and offline workflows

## Implementation Notes

**Agent Context Update**: The `.specify/scripts/powershell/update-agent-context.ps1` script could not be run due to PowerShell not being available in this environment. This script would normally update agent-specific context files with the new technology choices from this plan.

## Project Structure

### Documentation (this feature)

```text
specs/1-textbook-generation/
├── plan.md              # This file
├── research.md          # Phase 0 output
├── data-model.md        # Phase 1 output
├── quickstart.md        # Phase 1 output
├── contracts/           # Phase 1 output
│   └── textbook-generation-api.yaml
└── tasks.md             # Phase 2 output
```

### Source Code (repository root)

```text
backend/
├── src/
│   ├── models/
│   │   ├── textbook.py
│   │   ├── chapter.py
│   │   ├── section.py
│   │   ├── interactive_element.py
│   │   ├── user_profile.py
│   │   ├── chatbot_session.py
│   │   └── translation_cache.py
│   ├── services/
│   │   ├── content_generation/
│   │   │   ├── generation_service.py
│   │   │   ├── difficulty_service.py
│   │   │   ├── interactive_service.py
│   │   │   └── physical_ai_content_service.py
│   │   ├── textbook_structure/
│   │   │   ├── structure_service.py
│   │   │   ├── chapter_service.py
│   │   │   └── content_type_service.py
│   │   ├── export/
│   │   │   ├── export_service.py
│   │   │   ├── pdf_exporter.py
│   │   │   ├── html_exporter.py
│   │   │   └── docusaurus_exporter.py
│   │   ├── chatbot/
│   │   │   ├── rag_service.py
│   │   │   ├── vector_store_service.py
│   │   │   └── query_processor.py
│   │   ├── auth/
│   │   │   ├── user_service.py
│   │   │   └── background_capture_service.py
│   │   ├── personalization/
│   │   │   ├── personalization_service.py
│   │   │   └── content_adapter.py
│   │   ├── translation/
│   │   │   ├── translation_service.py
│   │   │   └── urdu_translator.py
│   │   ├── base_service.py
│   │   └── validation_service.py
│   ├── api/
│   │   ├── textbooks.py
│   │   ├── chatbot.py
│   │   ├── auth.py
│   │   ├── personalization.py
│   │   └── translation.py
│   ├── schemas/
│   │   ├── textbook_request.py
│   │   ├── textbook_response.py
│   │   ├── progress_response.py
│   │   ├── export_schemas.py
│   │   ├── chatbot_schemas.py
│   │   ├── auth_schemas.py
│   │   └── personalization_schemas.py
│   ├── utils/
│   │   ├── storage.py
│   │   ├── logger.py
│   │   └── security.py
│   └── exceptions.py
├── tests/
│   ├── test_content_generation.py
│   ├── test_chatbot.py
│   ├── test_auth.py
│   ├── test_personalization.py
│   └── test_translation.py
└── main.py

frontend/
├── src/
│   ├── components/
│   │   ├── textbook-generator/
│   │   │   ├── index.tsx
│   │   │   ├── input-form.tsx
│   │   │   └── structure-customizer.tsx
│   │   ├── preview/
│   │   │   ├── textbook-preview.tsx
│   │   │   └── chapter-preview.tsx
│   │   ├── chatbot/
│   │   │   ├── chatbot-widget.tsx
│   │   │   ├── chat-message.tsx
│   │   │   └── query-input.tsx
│   │   ├── auth/
│   │   │   ├── signup-form.tsx
│   │   │   ├── signin-form.tsx
│   │   │   └── background-questions.tsx
│   │   ├── personalization/
│   │   │   ├── personalization-toggle.tsx
│   │   │   └── content-adapter.tsx
│   │   ├── translation/
│   │   │   ├── translation-toggle.tsx
│   │   │   └── urdu-content.tsx
│   │   ├── export/
│   │   │   └── export-options.tsx
│   │   └── progress/
│   │       └── progress-tracker.tsx
│   ├── pages/
│   │   ├── textbook-generation.tsx
│   │   ├── textbook-preview.tsx
│   │   ├── chatbot-page.tsx
│   │   └── user-dashboard.tsx
│   ├── services/
│   │   ├── api-client.ts
│   │   ├── auth-service.ts
│   │   └── chatbot-service.ts
│   ├── types/
│   │   ├── textbook.ts
│   │   ├── user.ts
│   │   ├── chatbot.ts
│   │   └── personalization.ts
│   └── utils/
│       ├── validators.ts
│       └── helpers.ts
├── tests/
│   ├── components/
│   │   ├── textbook-generator.test.tsx
│   │   ├── chatbot.test.tsx
│   │   └── auth.test.tsx
│   └── services/
│       └── api-client.test.ts
├── package.json
├── tsconfig.json
└── vite.config.ts

docs/
├── textbooks/
│   └── [generated textbooks]
├── docusaurus.config.js
├── src/
│   ├── components/
│   │   ├── ChatbotWidget.js
│   │   └── PersonalizationToggle.js
│   ├── pages/
│   │   └── index.js
│   └── css/
│       └── custom.css
└── static/
    └── img/

website/
├── docusaurus.config.js
├── package.json
├── src/
│   ├── components/
│   ├── pages/
│   └── css/
└── static/
```

**Structure Decision**: Web application with separate backend API service and frontend UI, following modular architecture principle with clear separation of concerns. The backend handles content generation, RAG chatbot services, authentication, personalization, and translation, while the frontend provides the user interface and integrates with Docusaurus for textbook publishing to GitHub Pages.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| Multiple services | Required for clear separation of content generation, RAG chatbot, authentication, personalization, and translation | Tight coupling would violate modularity principle and make the system harder to maintain |
| External dependencies | RAG chatbot requires OpenAI Agents/ChatKit SDKs, Neon Postgres, and Qdrant for proper functionality | Self-built alternatives would be significantly more complex and less reliable than established services |