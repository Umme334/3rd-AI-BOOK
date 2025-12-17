# ADR 002: Urdu Translation System

## Context

The AI-Native Textbook for Physical AI and Humanoid Robotics needs to support Urdu translation to make the educational content accessible to Urdu-speaking students. This is particularly important for the hackathon context where inclusivity and accessibility are key requirements. The system must handle technical content related to robotics, AI, and engineering concepts in Urdu while maintaining accuracy and readability.

## Decision

Implement a comprehensive Urdu translation system using Google Cloud Translation API as the primary translation engine, with a specialized Urdu translator service that handles technical content. The system includes:
- TranslationRequest and TranslationResponse schemas for handling translation requests
- TranslationService for core translation logic with quality scoring
- UrduTranslator service specifically optimized for technical content translation
- TranslationCacheService for efficient caching of translations
- API endpoints for translation functionality
- Integration with the textbook model to track available translations

The system will support multiple translation levels (literal, contextual, adaptive) and include quality assessment mechanisms.

## Status

Accepted

## Consequences

### Positive
- Makes educational content accessible to Urdu-speaking students
- Supports the inclusive learning objectives of the Physical AI and Humanoid Robotics course
- Caching mechanism improves performance and reduces API costs
- Quality scoring helps maintain translation accuracy
- Modular design allows for adding other languages in the future

### Negative
- Adds dependency on Google Cloud Translation API for production use
- Increases system complexity with additional services and caching
- Technical terminology may not translate perfectly without domain-specific dictionaries
- Additional API costs for translation services

### Neutral
- Introduces new service layer specifically for translation functionality
- Adds translation-specific fields to the Textbook data model
- Requires Google Cloud credentials for production deployment

## Alternatives Considered

- Machine Translation API from other providers (AWS, Azure): Rejected in favor of Google's superior translation quality for Urdu
- Custom neural translation model: Rejected due to high development cost and maintenance requirements
- Manual translation by human translators: Rejected due to scalability and cost concerns
- Rule-based translation system: Rejected as it would not handle the complexity of technical content effectively

## Implementation Notes

- Translation quality is assessed based on length ratio between original and translated content
- Cache TTL is set to 24 hours by default to balance freshness and efficiency
- Urdu-specific handling for technical terminology in robotics and AI domains
- Integration with OpenAI client for fallback translation when Google API is unavailable
- Support for different translation levels (literal for direct translation, contextual for domain-aware translation)

## Links

- Related ADRs: ADR 001 (Content Personalization System)
- Related Specs: specs/1-textbook-generation/spec.md
- Implementation PRs: None yet