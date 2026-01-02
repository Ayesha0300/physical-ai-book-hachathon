# Data Model: Physical AI & Humanoid Robotics Book

**Feature**: 001-ros2-nervous-system | **Date**: 2025-12-24

## Content Data Model

This document defines the data structures and relationships for the Physical AI & Humanoid Robotics book project.

## Core Entities

### Module
- **id**: String (e.g., "module1-ros2", "module2-isaac")
- **title**: String (e.g., "The Robotic Nervous System (ROS 2)")
- **description**: String (brief overview of module content)
- **chapters**: Array<Chapter> (ordered list of chapters in the module)
- **prerequisites**: Array<String> (knowledge required before starting)
- **learning_objectives**: Array<String> (what students will learn)
- **duration**: Number (estimated time to complete in hours)
- **difficulty**: Enum ("beginner", "intermediate", "advanced")

### Chapter
- **id**: String (e.g., "chapter1-why-robots-need-nervous-system")
- **title**: String (e.g., "Why Robots Need a Nervous System")
- **module_id**: String (reference to parent module)
- **content**: String (markdown content of the chapter)
- **learning_objectives**: Array<String> (specific objectives for this chapter)
- **prerequisites**: Array<String> (what to know before this chapter)
- **exercises**: Array<Exercise> (practice problems for this chapter)
- **examples**: Array<Example> (code or conceptual examples)
- **diagrams**: Array<Diagram> (visual aids for understanding)
- **duration**: Number (estimated reading time in minutes)
- **position**: Number (order within the module)

### Exercise
- **id**: String (unique identifier)
- **title**: String (brief description of the exercise)
- **type**: Enum ("conceptual", "code", "simulation", "analysis")
- **difficulty**: Enum ("basic", "intermediate", "advanced")
- **content**: String (the exercise problem/description)
- **solution**: String (the solution or answer)
- **chapter_id**: String (reference to parent chapter)
- **expected_duration**: Number (time to complete in minutes)

### Example
- **id**: String (unique identifier)
- **title**: String (brief description)
- **type**: Enum ("code", "diagram", "simulation", "conceptual")
- **content**: String (the example content)
- **chapter_id**: String (reference to parent chapter)
- **explanation**: String (explanation of the example)

### Diagram
- **id**: String (unique identifier)
- **title**: String (description of the diagram)
- **type**: Enum ("architecture", "workflow", "conceptual", "code-flow")
- **file_path**: String (path to the diagram file)
- **alt_text**: String (description for accessibility)
- **chapter_id**: String (reference to parent chapter)
- **caption**: String (explanation of the diagram)

## Relationships

### Module-Chapter Relationship
- One Module contains many Chapters (1:M)
- Each Chapter belongs to exactly one Module
- Chapters are ordered within a Module

### Chapter-Exercise Relationship
- One Chapter contains many Exercises (1:M)
- Each Exercise belongs to exactly one Chapter

### Chapter-Example Relationship
- One Chapter contains many Examples (1:M)
- Each Example belongs to exactly one Chapter

### Chapter-Diagram Relationship
- One Chapter contains many Diagrams (1:M)
- Each Diagram belongs to exactly one Chapter

## Content Metadata

### Citation
- **id**: String (unique identifier)
- **type**: Enum ("book", "journal", "website", "documentation", "paper")
- **title**: String (title of the source)
- **author**: String (author or organization)
- **year**: Number (publication year)
- **url**: String (URL if applicable)
- **accessed_date**: Date (when the source was accessed)
- **apa_citation**: String (formatted APA citation)

### Concept
- **id**: String (unique identifier, e.g., "ros2-node", "isaac-sim", "gazebo-physics")
- **name**: String (concept name)
- **definition**: String (clear definition)
- **related_concepts**: Array<String> (other concept IDs that are related)
- **modules**: Array<String> (module IDs where this concept appears)
- **chapters**: Array<String> (chapter IDs where this concept appears)

## Validation Rules

### Content Validation
- All chapter content must be in valid Markdown format
- All code examples must be properly formatted with language specification
- All citations must follow APA format
- All diagrams must have alt text for accessibility
- All exercises must have solutions or clear solution paths

### Structural Validation
- Module IDs must follow the pattern "module[0-9]+-[a-z0-9-]+"
- Chapter IDs must follow the pattern "chapter[0-9]+-[a-z0-9-]+"
- Each module must have at least one chapter
- Chapters must be properly ordered within modules
- No circular dependencies between concepts

### Quality Validation
- Reading level must be grade 10-12 (Flesch-Kincaid)
- All technical claims must be verifiable
- No hallucinated APIs, tools, or capabilities
- Consistent terminology across all content
- All examples must be tested and functional

## Future Integration Data

### RAG Preparation
- **semantic_chunks**: Array<Chunk> (content chunks for AI retrieval)
  - **id**: String (unique chunk identifier)
  - **content**: String (chunked content)
  - **parent_id**: String (ID of parent entity)
  - **keywords**: Array<String> (relevant keywords for search)
  - **context**: String (surrounding context for chunk)

### Personalization Data
- **difficulty_adjustments**: Object (for adaptive learning paths)
- **prerequisite_mappings**: Object (for personalized learning sequences)
- **progress_tracking**: Object (for student progress monitoring)

## File Structure Mapping

The data model maps to the file system as follows:

```
docs/
├── [module.id]/
│   ├── [chapter.id].md
│   └── assets/
│       ├── diagrams/
│       │   └── [diagram.id].png
│       └── examples/
│           └── [example.id].py
├── _concepts/
│   └── [concept.id].md
├── _exercises/
│   └── [module.id]/
│       └── [exercise.id].md
└── _citations/
    └── [citation.id].md
```

This data model ensures consistent structure and relationships across the entire book while enabling future AI integration and personalization features.