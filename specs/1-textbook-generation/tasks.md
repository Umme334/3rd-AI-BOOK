
---
description: "Task list for AI-Native Textbook for Physical AI and Humanoid Robotics implementation"
---
# Tasks: AI-Native Textbook for Physical AI and Humanoid Robotics

**Input**: Design documents from `/specs/1-textbook-generation/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, data-model.md, contracts/

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [x] T001 Create project directory structure (backend/, frontend/, docs/, website/)
- [x] T002 Initialize backend project with FastAPI dependencies in backend/requirements.txt
- [x] T003 Initialize frontend project with React/TypeScript dependencies in frontend/package.json
- [x] T004 Configure environment variables for OpenAI API key, Neon Postgres, and Qdrant in .env

---
## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [x] T005 Create Textbook model in backend/src/models/textbook.py
- [x] T006 Create Chapter model in backend/src/models/chapter.py
- [x] T007 Create Section model in backend/src/models/section.py
- [x] T008 Create InteractiveElement model in backend/src/models/interactive_element.py
- [x] T009 Create UserProfile model in backend/src/models/user_profile.py for background capture
- [x] T010 Create ChatbotSession model in backend/src/models/chatbot_session.py
- [x] T011 Create TranslationCache model in backend/src/models/translation_cache.py
- [x] T012 Setup file storage utilities in backend/src/utils/storage.py
- [x] T013 Configure OpenAI client in backend/src/services/openai_client.py
- [x] T014 Configure Qdrant client for RAG vector storage in backend/src/services/qdrant_client.py
- [x] T015 Create API error handling framework in backend/src/exceptions.py
- [x] T016 Create base API service in backend/src/services/base_service.py
- [x] T017 Setup database connection for Neon Postgres in backend/src/database.py

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Generate Interactive Physical AI & Humanoid Robotics Content (Priority: P1) 🎯 MVP

**Goal**: Enable educators to generate interactive textbook content specifically for Physical AI and Humanoid Robotics using AI based on user parameters (subject, difficulty, length)

**Independent Test**: Can be fully tested by providing input parameters for Physical AI topics and verifying that it produces well-structured, educational content with interactive elements covering ROS 2, Gazebo, NVIDIA Isaac, and Vision-Language-Action modules

### Implementation for User Story 1

- [x] T018 Create TextbookCreateRequest schema in backend/src/schemas/textbook_request.py
- [x] T019 Create TextbookResponse schema in backend/src/schemas/textbook_response.py
- [x] T020 Create ProgressResponse schema in backend/src/schemas/progress_response.py
- [x] T021 Implement TextbookService for creation in backend/src/services/textbook_service.py
- [x] T022 Implement ContentGenerationService in backend/src/services/content_generation/generation_service.py
- [x] T023 Implement PhysicalAIContentService for ROS 2, Gazebo, NVIDIA Isaac content in backend/src/services/content_generation/physical_ai_content_service.py
- [x] T024 Implement difficulty-based content customization for hardware backgrounds in backend/src/services/content_generation/difficulty_service.py
- [x] T025 Create textbook creation endpoint in backend/src/api/textbooks.py
- [x] T026 Create textbook generation endpoint in backend/src/api/textbooks.py
- [x] T027 Create textbook progress endpoint in backend/src/api/textbooks.py
- [x] T028 Create textbook preview endpoint in backend/src/api/textbooks.py
- [x] T029 Implement content validation and input parameter validation in backend/src/services/validation_service.py
- [x] T030 Add interactive element generation to content service in backend/src/services/content_generation/interactive_service.py

**Checkpoint**: At this point, User Story 1 should be fully functional and testable independently

---

## Phase 4: User Story 2 - RAG Chatbot for Physical AI Content (Priority: P1)

**Goal**: Enable students to interact with a RAG chatbot that answers questions about Physical AI & Humanoid Robotics content based only on selected text from the book

**Independent Test**: Can be fully tested by asking the RAG chatbot questions about Physical AI content and verifying that it provides accurate answers based only on the selected text

### Implementation for User Story 2

- [x] T031 Create ChatbotQueryRequest and ChatbotResponse schemas in backend/src/schemas/chatbot_schemas.py
- [x] T032 Implement RAGService for question answering in backend/src/services/chatbot/rag_service.py
- [x] T033 Implement VectorStoreService using Qdrant for content indexing in backend/src/services/chatbot/vector_store_service.py
- [x] T034 Implement QueryProcessor for natural language processing in backend/src/services/chatbot/query_processor.py
- [x] T035 Create chatbot query endpoint in backend/src/api/chatbot.py
- [x] T036 Create chatbot session management endpoint in backend/src/api/chatbot.py
- [x] T037 Implement content chunking for RAG indexing in backend/src/services/chatbot/content_chunker.py
- [x] T038 Update Textbook model to support RAG indexing in backend/src/models/textbook.py

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently

---

## Phase 5: User Story 3 - User Authentication and Background Capture (Priority: P2)

**Goal**: Enable students to sign up with better-auth.com and capture their software and hardware background information

**Independent Test**: Can be tested by registering a new user and verifying that their background information is captured and stored

### Implementation for User Story 3

- [x] T039 Create AuthRequest and AuthResponse schemas in backend/src/schemas/auth_schemas.py
- [x] T040 Implement UserService for user management in backend/src/services/auth/user_service.py
- [x] T041 Implement BackgroundCaptureService for collecting user experience data in backend/src/services/auth/background_capture_service.py
- [x] T042 Create signup endpoint with background questions in backend/src/api/auth.py
- [x] T043 Create signin endpoint in backend/src/api/auth.py
- [x] T044 Create user profile management endpoint in backend/src/api/auth.py
- [x] T045 Configure better-auth.com integration in backend/src/api/auth.py

**Checkpoint**: At this point, User Stories 1, 2 AND 3 should all work independently

---

## Phase 6: User Story 4 - Content Personalization (Priority: P2)

**Goal**: Allow logged users to personalize textbook content based on their software and hardware background

**Independent Test**: Can be tested by using personalization features and verifying that content adapts to the user's technical background

### Implementation for User Story 4

- [x] T046 Create PersonalizationRequest schema in backend/src/schemas/personalization_schemas.py
- [x] T047 Implement PersonalizationService for content adaptation in backend/src/services/personalization/personalization_service.py
- [x] T048 Implement ContentAdapter for modifying content based on user profile in backend/src/services/personalization/content_adapter.py
- [x] T049 Create personalization toggle endpoint in backend/src/api/personalization.py
- [x] T050 Update Textbook model to support personalization in backend/src/models/textbook.py
- [x] T051 Create personalization preferences endpoint in backend/src/api/personalization.py

**Checkpoint**: At this point, User Stories 1, 2, 3 AND 4 should all work independently

---

## Phase 7: User Story 5 - Urdu Translation (Priority: P3)

**Goal**: Allow logged users to translate textbook content to Urdu by chapter

**Independent Test**: Can be tested by using translation features and verifying that content is accurately translated to Urdu while maintaining technical accuracy

### Implementation for User Story 5

- [x] T052 Create TranslationRequest schema in backend/src/schemas/translation_schemas.py
- [x] T053 Implement TranslationService for content translation in backend/src/services/translation/translation_service.py
- [x] T054 Implement UrduTranslator for technical content translation in backend/src/services/translation/urdu_translator.py
- [x] T055 Create translation endpoint in backend/src/api/translation.py
- [x] T056 Create translation cache management in backend/src/services/translation/translation_cache_service.py
- [x] T057 Update Textbook model to support translation caching in backend/src/models/textbook.py

**Checkpoint**: At this point, User Stories 1, 2, 3, 4 AND 5 should all work independently

---

## Phase 8: User Story 6 - Customize Textbook Structure (Priority: P2)

**Goal**: Allow educators to customize the structure and format of the generated textbook to match Physical AI & Humanoid Robotics curriculum requirements

**Independent Test**: Can be tested by configuring different textbook structures and verifying the output matches the specified structure with modules for ROS 2, Gazebo, NVIDIA Isaac, and Vision-Language-Action

### Implementation for User Story 6

- [x] T058 Create TextbookStructureRequest schema in backend/src/schemas/textbook_request.py
- [x] T059 Implement TextbookStructureService in backend/src/services/textbook_structure/structure_service.py
- [x] T060 Implement chapter organization customization in backend/src/services/textbook_structure/chapter_service.py
- [x] T061 Implement content type selection (text, images, quizzes) in backend/src/services/textbook_structure/content_type_service.py
- [x] T062 Add structure customization endpoint to backend/src/api/textbooks.py
- [x] T063 Update Textbook model to support custom structures in backend/src/models/textbook.py

**Checkpoint**: At this point, all user stories should work independently

---

## Phase 9: User Story 7 - Export to Docusaurus for GitHub Pages (Priority: P3)

**Goal**: Allow educators to export the generated textbook in Docusaurus format for deployment to GitHub Pages

**Independent Test**: Can be tested by generating a textbook and verifying successful export in Docusaurus format and deployment to GitHub Pages

### Implementation for User Story 7

- [x] T064 Create ExportRequest and ExportResponse schemas in backend/src/schemas/export_schemas.py
- [x] T065 Implement ExportService in backend/src/services/export/export_service.py
- [x] T066 Implement PDF export functionality in backend/src/services/export/pdf_exporter.py
- [x] T067 Implement HTML export functionality in backend/src/services/export/html_exporter.py
- [x] T068 Implement Docusaurus export functionality in backend/src/services/export/docusaurus_exporter.py
- [x] T069 Create export endpoint in backend/src/api/textbooks.py
- [x] T070 Implement file download handling in backend/src/api/textbooks.py
- [x] T071 Update Textbook model to track export formats in backend/src/models/textbook.py

**Checkpoint**: All user stories should now be independently functional

---

## Phase 10: Frontend Implementation for All Stories

**Goal**: Create web interface for textbook generation, RAG chatbot, authentication, personalization, translation, and export

- [x] T072 Create main textbook generator component in frontend/src/components/textbook-generator/index.tsx
- [x] T073 Create textbook input form in frontend/src/components/textbook-generator/input-form.tsx
- [x] T074 Create textbook preview component in frontend/src/components/preview/textbook-preview.tsx
- [x] T075 Create export options component in frontend/src/components/export/export-options.tsx
- [x] T076 Create progress tracking component in frontend/src/components/progress/progress-tracker.tsx
- [x] T077 Create structure customization component in frontend/src/components/textbook-generator/structure-customizer.tsx
- [x] T078 Create RAG chatbot widget component in frontend/src/components/chatbot/chatbot-widget.tsx
- [x] T079 Create chat message component in frontend/src/components/chatbot/chat-message.tsx
- [x] T080 Create query input component for chatbot in frontend/src/components/chatbot/query-input.tsx
- [x] T081 Create signup form with background questions in frontend/src/components/auth/signup-form.tsx
- [x] T082 Create signin form in frontend/src/components/auth/signin-form.tsx
- [x] T083 Create background questions component in frontend/src/components/auth/background-questions.tsx
- [x] T084 Create personalization toggle component in frontend/src/components/personalization/personalization-toggle.tsx
- [x] T085 Create translation toggle component in frontend/src/components/translation/translation-toggle.tsx
- [x] T086 Create Urdu content display component in frontend/src/components/translation/urdu-content.tsx
- [x] T087 Connect frontend to backend API in frontend/src/services/api-client.ts
- [x] T088 Create main page for textbook generation in frontend/src/pages/textbook-generation.tsx
- [x] T089 Create chatbot page in frontend/src/pages/chatbot-page.tsx
- [x] T090 Create user dashboard page in frontend/src/pages/user-dashboard.tsx

---

## Phase 11: Docusaurus Integration and GitHub Pages Deployment

**Goal**: Integrate with Docusaurus for textbook publishing and deploy to GitHub Pages

- [x] T091 Configure docusaurus.config.js for textbook publishing
- [x] T092 Create Docusaurus components for chatbot integration in docs/src/components/
- [x] T093 Create Docusaurus pages for textbook navigation in docs/src/pages/
- [x] T094 Implement GitHub Pages deployment workflow in .github/workflows/
- [x] T095 Create static assets for textbook in docs/static/
- [x] T096 Integrate chatbot widget into Docusaurus pages
- [x] T097 Implement personalization features in Docusaurus components
- [x] T098 Implement translation features in Docusaurus components

---

## Phase 12: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

- [x] T099 Add unit tests for backend services in backend/tests/
- [x] T100 Add integration tests for API endpoints in backend/tests/
- [x] T101 Add frontend component tests in frontend/tests/
- [x] T102 Add authentication middleware in backend/src/middleware/auth.py
- [x] T103 Implement rate limiting for generation requests in backend/src/middleware/rate_limiter.py
- [x] T104 Add logging framework to backend services in backend/src/utils/logger.py
- [x] T105 Update documentation in docs/
- [x] T106 Run quickstart.md validation
- [x] T107 Add security middleware for input validation in backend/src/middleware/security.py
- [x] T108 Implement caching for translation and personalization in backend/src/middleware/cache.py

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3+)**: All depend on Foundational phase completion
  - User stories can then proceed in parallel (if staffed)
  - Or sequentially in priority order (P1 → P2 → P3)
- **Frontend (Phase 10)**: Depends on API endpoints being available
- **Docusaurus Integration (Phase 11)**: Depends on export functionality being complete
- **Polish (Final Phase)**: Depends on all desired user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 2 (P1)**: Depends on User Story 1 (needs content to index) - May integrate with US1 but should be independently testable
- **User Story 3 (P2)**: Can start after Foundational (Phase 2) - May integrate with other stories but should be independently testable
- **User Story 4 (P2)**: Depends on User Story 3 (needs user profiles) - May integrate with US1 but should be independently testable
- **User Story 5 (P3)**: Depends on User Story 3 (needs authentication) - May integrate with US1 but should be independently testable
- **User Story 6 (P2)**: Can start after Foundational (Phase 2) - May integrate with US1 but should be independently testable
- **User Story 7 (P3)**: Can start after Foundational (Phase 2) - May integrate with US1/US6 but should be independently testable

### Within Each User Story

- Models before services
- Services before endpoints
- Core implementation before integration
- Story complete before moving to next priority

### Parallel Opportunities

- All Setup tasks marked [P] can run in parallel
- All Foundational tasks marked [P] can run in parallel (within Phase 2)
- Once Foundational phase completes, User Stories 1, 3, 6, and 7 can start in parallel
- User Stories 2 and 4 can start after US1 and US3 respectively
- User Story 5 can start after US3
- Frontend components can be developed in parallel after API availability
- Docusaurus integration can happen in parallel with frontend development

## Implementation Strategy

### MVP First (User Story 1 + 2 Only)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational (CRITICAL - blocks all stories)
3. Complete Phase 3: User Story 1 (Physical AI content generation)
4. Complete Phase 4: User Story 2 (RAG chatbot)
5. Complete Phase 10: Frontend Implementation (basic for US1 and US2)
6. **STOP and VALIDATE**: Test User Stories 1 and 2 independently
7. Deploy/demo if ready

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready
2. Add User Story 1 → Test independently → Deploy/Demo (MVP!)
3. Add User Story 2 → Test independently → Deploy/Demo
4. Add User Story 3 → Test independently → Deploy/Demo
5. Add User Story 4 → Test independently → Deploy/Demo
6. Add User Story 5 → Test independently → Deploy/Demo
7. Add User Stories 6 & 7 → Test independently → Deploy/Demo
8. Add Frontend → Test full workflow → Deploy/Demo
9. Add Docusaurus Integration → Deploy to GitHub Pages
10. Each story adds value without breaking previous stories

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup + Foundational together
2. Once Foundational is done:
   - Developer A: User Story 1 (Physical AI content)
   - Developer B: User Story 2 (RAG chatbot)
   - Developer C: User Story 3 (Authentication)
3. After US1: Developer D starts User Story 4 (Personalization)
4. After US3: Developer E starts User Story 5 (Translation)
5. Frontend developers: Implement UI components in parallel
6. DevOps engineer: Handle Docusaurus integration and deployment
7. Stories complete and integrate independently
