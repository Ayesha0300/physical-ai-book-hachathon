# Data Model: RAG Content Ingestion Pipeline

**Feature**: RAG Content Ingestion Pipeline
**Date**: 2026-01-04
**Branch**: 001-rag-pipeline

## Overview

This document defines the data structures used in the RAG content ingestion pipeline, including content representation, metadata schemas, and vector storage formats.

## Content Data Structures

### ContentChunk
Represents a segment of extracted content that will be converted to embeddings.

```
ContentChunk {
  id: string (UUID) - unique identifier for the chunk
  text: string - the actual content text
  source_url: string - URL where the content originated
  chunk_index: integer - position of this chunk in the original document
  metadata: object - additional information about the content
    - title: string - page title
    - headings: array - hierarchy of headings in the chunk
    - content_type: string - type of content (text, code, etc.)
    - processed_at: timestamp - when the chunk was created
}
```

### ExtractedPage
Represents a fully extracted web page before chunking.

```
ExtractedPage {
  url: string - the source URL
  title: string - page title
  content: string - full text content of the page
  headings: array - list of all headings in the page
  links: array - internal/external links found in the page
  extracted_at: timestamp - when the page was extracted
  content_hash: string - hash of content for change detection
}
```

## Vector Database Schema

### Qdrant Collection Structure
The vector database will store content chunks with associated metadata.

```
Qdrant Point {
  id: UUID - unique identifier for the point
  vector: array<number> - embedding vector from Cohere
  payload: object - metadata associated with the vector
    - text: string - the original text content
    - source_url: string - URL where content originated
    - title: string - page title
    - chunk_index: integer - position in original document
    - headings: array - headings hierarchy in the chunk
    - processed_at: timestamp - when chunk was processed
}
```

## Configuration Structure

### Environment Variables
All configuration is loaded from environment variables.

```
Configuration {
  COHERE_API_KEY: string - API key for Cohere service
  QDRANT_URL: string - URL for Qdrant Cloud instance
  QDRANT_API_KEY: string - API key for Qdrant Cloud
  QDRANT_COLLECTION_NAME: string - name of the collection to use
  CHUNK_SIZE: integer - maximum size of content chunks (default: 512)
  CHUNK_OVERLAP: integer - overlap between chunks (default: 50)
  REQUEST_TIMEOUT: integer - timeout for HTTP requests (default: 30)
  BATCH_SIZE: integer - number of items to process in each batch (default: 10)
  SITEMAP_URL: string - URL of the sitemap.xml to process
}
```

## Processing Pipeline Data Flow

### Data Flow Sequence
1. **Sitemap Fetch** → List of URLs
2. **URL Fetch** → Raw HTML content
3. **HTML Parse** → ExtractedPage objects
4. **Content Chunk** → ContentChunk objects
5. **Embed Generation** → ContentChunk with embedding
6. **Vector Store** → Stored in Qdrant collection

### Metadata Preservation
Each step preserves and enriches metadata:
- Source URL is maintained throughout the pipeline
- Document structure (headings) is preserved
- Processing timestamps are added
- Content relationships are maintained

## Validation Rules

### ContentChunk Validation
- Text must not be empty
- Source URL must be a valid URL
- Chunk index must be non-negative
- Text length must be within configured limits

### Vector Requirements
- Embedding vector must match expected dimension from Cohere model
- Payload must contain required metadata fields
- Point ID must be unique within collection

## Error Handling Data

### Error Log Structure
```
ProcessingError {
  error_id: string - unique identifier for the error
  stage: string - pipeline stage where error occurred
  url: string - URL that caused the error (if applicable)
  error_type: string - type of error that occurred
  error_message: string - detailed error message
  occurred_at: timestamp - when error occurred
  payload: object - additional context about the error
}
```