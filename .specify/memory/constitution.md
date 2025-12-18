<!-- SYNC IMPACT REPORT
Version change: N/A (initial version) → 1.0.1
Modified principles: Added Technical Stack, Feature, and Course Content Requirements
Added sections: Technical Stack Requirements, Feature Requirements, Course Content Requirements
Removed sections: N/A
Templates requiring updates: N/A
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
Students engage via chatbot, personalization, and translation; RAG chatbot answers questions from book content only; Rich interactive elements enhance learning

### Cloud + Edge Hybrid Approach
Training in cloud, deployment on Jetson kits; Support cloud-native labs for students without RTX-enabled PCs; Balance between accessibility and advanced capabilities

### Minimal Dependencies
Must avoid complexity and heavy dependencies; Prefer lightweight solutions that work on limited resources; Reduce friction for deployment and maintenance

## Technical Stack Requirements
- Docusaurus for book creation and documentation
- GitHub Pages for deployment
- RAG chatbot using OpenAI Agents/ChatKit SDKs, FastAPI, Neon Serverless Postgres, and Qdrant Cloud Free Tier
- Better-auth.com for user authentication and authorization
- Support for user background capture during signup process

## Feature Requirements
- Integrated RAG chatbot that answers questions based only on selected text from the book
- User authentication with signup/signin using better-auth.com
- Background questions during signup to capture user's software and hardware experience
- Personalized content by chapter based on user's background
- Urdu translation of content by chapter
- Support for Physical AI & Humanoid Robotics course content as specified in the hackathon requirements

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
Must work on free tiers (Qdrant + Neon); Demo video ≤90 seconds; Must support low-end devices (users reading on phones); Must avoid complexity and heavy dependencies

## User Experience Requirements
Clean UI, fast loading, mobile friendly; Book is publicly accessible and fully functional; RAG chatbot answers correctly and contextually; Signup/Signin works with background capture; Personalization and Urdu translation buttons operate seamlessly

## Governance
Constitution supersedes all other practices; All implementations must verify compliance with modularity, scalability, accessibility principles; Complexity must be justified; Architecture decisions must align with principles; Use this constitution for guidance on development decisions

**Version**: 1.0.1 | **Ratified**: 2025-12-10 | **Last Amended**: 2025-12-13