# Data Model: RAG Pipeline Validation

## Entities

### Query
- **Fields**:
  - text: string (the natural language query to validate)
  - timestamp: datetime (when the query was executed)
- **Validation rules**: Must not be empty, should be natural language text

### RetrievedChunk
- **Fields**:
  - content: string (the text content retrieved from the vector database)
  - similarity_score: float (how well the chunk matches the query, 0.0-1.0)
  - metadata: object (URL, section, chunk_id, and other source information)
  - rank: integer (position in the ranked results list)
- **Validation rules**: Content must not be empty, similarity_score must be between 0.0 and 1.0

### Metadata
- **Fields**:
  - url: string (source URL of the content)
  - section: string (section identifier in the source)
  - chunk_id: string (unique identifier for the chunk)
  - source_file: string (original file name if applicable)
- **Validation rules**: URL must be valid, chunk_id must be unique

### ValidationResult
- **Fields**:
  - query: Query (the original query that was validated)
  - retrieved_chunks: array[RetrievedChunk] (the top-k results from the search)
  - execution_time: float (time taken to execute the query in seconds)
  - success: boolean (whether the query was successful)
  - error_message: string (if any error occurred)
- **Validation rules**: execution_time must be positive, success must align with error_message

### ValidationMetrics
- **Fields**:
  - success_rate: float (percentage of successful queries)
  - avg_latency: float (average time to retrieve results)
  - avg_similarity: float (average similarity score of results)
  - relevance_score: float (estimated relevance of results)
- **Validation rules**: All metrics must be between 0.0 and 1.0 where applicable