# Data Model: Textbook Generation

## Entities

### Textbook
- **id**: string (unique identifier)
- **title**: string (title of the textbook)
- **subject**: string (subject area of the textbook)
- **difficulty**: enum (beginner | intermediate | advanced)
- **target_audience**: string (description of target audience)
- **chapters**: array of Chapter objects
- **metadata**: object (creation date, author, word count, etc.)
- **export_formats**: array of strings (available export formats)
- **status**: enum (draft | generating | complete | failed)

### Chapter
- **id**: string (unique identifier)
- **title**: string (chapter title)
- **position**: integer (order in textbook)
- **sections**: array of Section objects
- **word_count**: integer (estimated word count)
- **learning_objectives**: array of strings (learning objectives for this chapter)

### Section
- **id**: string (unique identifier)
- **title**: string (section title)
- **content**: string (main content of the section)
- **position**: integer (order in chapter)
- **interactive_elements**: array of InteractiveElement objects
- **key_terms**: array of strings (key terms defined in this section)

### InteractiveElement
- **id**: string (unique identifier)
- **type**: enum (quiz | summary | booster | exercise)
- **content**: string (the actual interactive content)
- **position**: integer (position within parent section)
- **metadata**: object (type-specific metadata)

### UserPreferences
- **id**: string (user identifier)
- **default_difficulty**: enum (beginner | intermediate | advanced)
- **preferred_export_format**: string (default export format)
- **custom_subjects**: array of strings (subjects user frequently generates)
- **export_history**: array of objects (tracking previous exports)

## Relationships
- Textbook 1 --- * Chapter (one textbook has many chapters)
- Chapter 1 --- * Section (one chapter has many sections)
- Section 1 --- * InteractiveElement (one section has many interactive elements)

## Validation Rules
- Textbook title: required, 1-200 characters
- Subject: required, must be from approved list or custom with validation
- Difficulty: required, must be one of the defined enum values
- Chapter title: required, 1-100 characters
- Section content: required, minimum 50 characters
- Export formats: must be one of supported formats (pdf, html, markdown)

## State Transitions
- Textbook: draft → generating → complete OR failed
- User can regenerate content while in "complete" state
- Export formats can be added to "complete" textbooks