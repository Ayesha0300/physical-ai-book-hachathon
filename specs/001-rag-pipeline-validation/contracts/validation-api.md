# API Contract: RAG Pipeline Validation

## Input Contract

### Query Parameters
- `query`: string (required) - The natural language query to validate against the retrieval pipeline
- `top_k`: integer (optional, default: 5) - Number of top results to retrieve
- `collection_name`: string (optional, default: "book_chunks") - Name of the Qdrant collection to search

## Output Contract

### Success Response
```json
{
  "query": "the input query string",
  "execution_time": 0.123,
  "success": true,
  "retrieved_chunks": [
    {
      "content": "text content of the retrieved chunk",
      "similarity_score": 0.876,
      "rank": 1,
      "metadata": {
        "url": "source URL",
        "section": "section identifier",
        "chunk_id": "unique chunk identifier"
      }
    }
  ],
  "validation_metrics": {
    "success_rate": 1.0,
    "avg_latency": 0.123,
    "avg_similarity": 0.876
  }
}
```

### Error Response
```json
{
  "query": "the input query string",
  "success": false,
  "error_message": "descriptive error message",
  "execution_time": 0.010
}
```

## Validation Contract

The system guarantees:
- All retrieved chunks will have content, similarity score, and metadata
- Execution time will be measured in seconds
- Similarity scores will be between 0.0 and 1.0
- Top-k results will not exceed the requested number
- Metadata fields will be preserved as stored in the vector database