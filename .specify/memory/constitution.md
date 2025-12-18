<!-- SYNC IMPACT REPORT
Version change: 1.0.1 → 1.1.0
Modified principles: Enhanced Interactive Learning Focus to include sub-agent architecture and skills framework
Added sections: Sub-Agent Architecture Requirements, Skills Framework Requirements
Removed sections: N/A
Templates requiring updates: .specify/templates/plan-template.md, .specify/templates/spec-template.md, .specify/templates/tasks-template.md
Follow-up TODOs: None
-->

# AI-Native Textbook for Physical AI and Humanoid Robotics Constitution

## Core Principles

### Modularity First
Every feature (auth, personalization, translation, chatbot) is independent; Components must be self-contained, reusable, and independently testable; Clear interfaces required for all modules

### Scalability Design
Architecture designed to extend to other AI-native textbooks; System must support growth in users, content, and features; Future-proofing through clean abstractions

### Accessibility & Performance (NON-NEGOTIABLE)
Works on GitHub Pages/Vercel with minimal setup; Must work on free tiers (Qdrant + Neon); Clean UI, fast loading, mobile friendly; Must support low-end devices (users reading on phones)

### Interactive Learning Focus
Students engage via chatbot, personalization, and translation; RAG chatbot answers questions from book content only; Rich interactive elements enhance learning; Sub-agent architecture enables sophisticated content processing; Skills framework provides modular extensibility

### Cloud + Edge Hybrid Approach
Training in cloud, deployment on Jetson kits; Support cloud-native labs for students without RTX-enabled PCs; Balance between accessibility and advanced capabilities

### Minimal Dependencies
Must avoid complexity and heavy dependencies; Prefer lightweight solutions that work on limited resources; Reduce friction for deployment and maintenance

## Technical Stack Requirements
- Docusaurus for book creation and documentation
- GitHub Pages for deployment
- RAG chatbot using FastAPI, Neon Serverless Postgres, and Qdrant Cloud Free Tier
- Google Cloud APIs for content generation and translation (primary), with OpenAI as fallback
- Sub-agent architecture with coordinator, retrieval, and generation agents
- Skills framework for modular functionality
- Better-auth.com for user authentication and authorization
- Support for user background capture during signup process

## Feature Requirements
- Integrated RAG chatbot that answers questions based only on selected text from the book
- Sub-agent architecture with coordinator, retrieval, and generation agents for enhanced processing
- Skills framework enabling modular extensibility of functionality
- User authentication with signup/signin using better-auth.com
- Background questions during signup to capture user's software and hardware experience
- Personalized content by chapter based on user's background
- Urdu translation of content by chapter
- Support for Physical AI & Humanoid Robotics course content as specified in the hackathon requirements
- Text selection and context-aware querying capabilities

## Sub-Agent Architecture Requirements
- Coordinator Agent: Manages overall RAG workflow and agent communication
- Retrieval Agent: Handles context retrieval from vector store
- Generation Agent: Processes responses using LLM with retrieved context
- Routing Agent: Directs messages between agents for optimal processing
- All agents must support asynchronous communication and error handling
- Agents must be independently testable and maintainable

## Skills Framework Requirements
- Text Retrieval Skill: Finds relevant content from indexed textbooks
- Content Indexing Skill: Indexes textbook content for RAG search
- Text Generation Skill: Generates contextually appropriate responses
- Content Chunking Skill: Breaks content into searchable chunks
- Content Analysis Skill: Analyzes and processes content for optimization
- Skills must be modular, reusable, and independently testable
- Skills framework must support skill chaining and batch execution

## Course Content Requirements
- Physical AI principles and embodied intelligence
- ROS 2 (Robot Operating System) for robotic control
- Robot simulation with Gazebo and Unity
- NVIDIA Isaac AI robot platform
- Humanoid robots for natural interactions
- Integration of GPT models for conversational robotics
- Weekly breakdown covering 13 weeks of content from introduction to Physical AI to conversational robotics
- Hardware requirements documentation for Digital Twin Workstation, Edge Kit, and Robot Lab

## Development Constraints
Must work on free tiers (Qdrant + Neon); Demo video ≤90 seconds; Must support low-end devices (users reading on phones); Must avoid complexity and heavy dependencies; Sub-agent architecture must be lightweight and efficient; Skills framework must support modular testing and deployment

## User Experience Requirements
Clean UI, fast loading, mobile friendly; Book is publicly accessible and fully functional; RAG chatbot answers correctly and contextually; Signup/Signin works with background capture; Personalization and Urdu translation buttons operate seamlessly; Text selection and focused querying capabilities available; Chatbot provides source attribution and confidence scoring

## Governance
Constitution supersedes all other practices; All implementations must verify compliance with modularity, scalability, accessibility principles; Complexity must be justified; Architecture decisions must align with principles; Use this constitution for guidance on development decisions; Sub-agent architecture and skills framework implementations must follow defined interfaces and communication patterns

**Version**: 1.1.0 | **Ratified**: 2025-12-10 | **Last Amended**: 2025-12-22