---
id: 1
title: Create-project-constitution-for-AI-textbook
stage: constitution
date: 2025-12-10
surface: agent
model: Claude Sonnet 4.5
feature: constitution
branch: main
user: user
command: /sp.constitution
labels: ["constitution", "project-setup", "governance"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - .specify/memory/constitution.md
tests:
 - none
---

## Prompt

```text
project constitution: AI-Native Textbook for physical AI and Humanoid Robotics

---

## 🎯 Mission
build a fully AI-native, interactive, intellegent textbook that teaches the Physical AI & Humanoid Robotics course.
To design and publish an **AI-native textbook** on *Physical AI & Humanoid Robotics* that integrates interactive learning, personalization, and embodied intelligence. The project will serve as a foundation for Panaversity's ecosystem, enabling students to learn cutting-edge robotics through simulation, edge deployment, and conversational AI.

---

## 📦 Core Deliverables
1. **AI/Spec-Driven Book Creation**
   - Textbook authored in Docusaurus using Spec-Kit Plus + Claude Code.
     a docursaurus-based interactive textbook  with 6-8 short, clean, modern chapters.
     a fully functional RAG chatbot answerimg questions only from the book.
     user authentication (signup/ login) using Better-Auth.
     personalization chapter content based on user background.
     one-click Urdu translation for every chapter.
     auto-generated summarize, quizzes,  and learning boosters.


2. **Integrated RAG Chatbot**
   - Retrieval-Augmented Generation chatbot embedded in the book.
   - Powered by FastAPI, Neon Postgres, Qdrant Cloud, and OpenAI Agents/ChatKit.
   - Supports contextual Q&A and "answer from selected text only."

3. **Bonus Features (up to 150 pts)**
   - Signup/Signin with BetterAuth, capturing user background.
   - Personalization button per chapter (content tailored to user profile).
   - Urdu translation toggle per chapter.
   - Claude Code Subagents (Teaching Assistant, Lab Mentor, Exam Prep).

---

## ✅ Success Criteria
⦁    clean UI , fast loading, mobile friendly.
- Book is **publicly accessible** and fully functional.
- RAG chatbot answers correctly and contextually.
- Signup/Signin works with background capture.
- Personalization and Urdu translation buttons operate seamlessly.
- Demo video (≤90 seconds) clearly showcases features.
- Architecture supports scalability and future AI-native textbooks.

---

## 🚫 Non-Goals
- Building a full humanoid robot hardware platform.
- Supporting non-course-related AI domains (e.g., finance, gaming).
- Providing commercial-grade cloud robotics infrastructure beyond demo scope.
- Long-form video tutorials (judges only watch first 90 seconds).

---

## 🏛️ Architecture Principles
- **Modularity:** Each feature (auth, personalization, translation, chatbot) is independent.
- **Scalability:** Designed to extend to other AI-native textbooks.
- **Accessibility:** Works on GitHub Pages/Vercel with minimal setup.
- **Interactivity:** Students engage via chatbot, personalization, and translation.
- **Cloud + Edge Hybrid:** Training in cloud, deployment on Jetson kits.

---

## 👥 User Stories (Prioritized)
1. **As a student**, I want to read the textbook online so I can learn Physical AI.
2. **As a student**, I want to ask the chatbot questions about the book so I can clarify concepts.
3. **As a student**, I want to personalize chapters based on my background so content matches my skill level.
4. **As a student**, I want to translate chapters into Urdu so I can learn in my native language.
5. **As an instructor**, I want to see students' personalized flows so I can guide them better.
6. **As a developer**, I want modular architecture so I can reuse components for other AI-native books.

---

## ⚖️ Constraints
- must work on free tiers (drant + Neon).
- Demo video ≤90 seconds.
- must support low-end devices (users reading on phones).
- must avoid complexity and heavy dependencies.
---

## ⚠️ Risks & Mitigation
- **Risk:** Latency in cloud-to-robot control.
  - *Mitigation:* Train in cloud, deploy weights locally on Jetson.
- **Risk:** Students lack RTX-enabled PCs.
  - *Mitigation:* Provide cloud-native lab option.
- **Risk:** Urdu translation inaccuracies.
  - *Mitigation:* Use hybrid approach (pre-translated + API fallback).
- **Risk:** Personalization complexity.
  - *Mitigation:* Start with simple beginner/advanced toggles, expand later.

---

## 🏁 Definition of Done
- all chapters visible  and readable.
-quizzes+ summarise per chapter generted.
- RAG chatbot embedded and functional.
- Signup/Signin with background capture implemented.
- Personalization and Urdu translation buttons working.
- Demo video (≤90 seconds).
-fully deployed URLs live and stable.

--
```

## Response snapshot

The project constitution for the AI-Native Textbook for Physical AI and Humanoid Robotics has been successfully created and updated. Here's the summary:

## Final Summary

**New Version**: 1.0.0 (initial version)
**Bump Rationale**: This is the initial constitution for the project, establishing core principles and governance for the AI-Native Textbook project.

**Files Updated**:
- `.specify/memory/constitution.md` - The main constitution file has been created with all project principles

**Constitution Overview**:
The constitution establishes 6 core principles for the project:
1. Modularity First - Features must be independent and self-contained
2. Scalability Design - Architecture must support growth and extensibility
3. Accessibility & Performance - Must work on limited resources and be performant
4. Interactive Learning Focus - Student engagement through chatbot and personalization
5. Cloud + Edge Hybrid Approach - Training in cloud, deployment on edge devices
6. Minimal Dependencies - Reduce complexity and deployment friction

**Suggested Commit Message**:
`docs: add initial constitution for AI-Native Textbook project v1.0.0`

The constitution aligns with the project requirements specified by the user, emphasizing modularity, scalability, accessibility, and interactive learning as required in the original specification.

## Outcome

- ✅ Impact: Project constitution successfully created with 6 core principles aligned to project requirements
- 🧪 Tests: No tests required for constitution creation
- 📁 Files: .specify/memory/constitution.md updated with complete constitution
- 🔁 Next prompts: Project constitution is complete, next steps would be feature specification
- 🧠 Reflection: Constitution establishes clear governance and principles for the AI textbook project

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): PASS - Constitution successfully created and written to file
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Begin feature specification based on constitution