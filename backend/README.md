# RAG Pipeline Validation Tool

This tool validates the RAG (Retrieval-Augmented Generation) pipeline by testing the retrieval component. It connects to a Qdrant vector database, performs semantic search with Cohere embeddings, and validates the quality of retrieved results.

## Features

- **Single Query Validation**: Validate a single natural language query against the retrieval pipeline
- **Batch Query Validation**: Execute multiple queries and analyze results collectively
- **Quality Metrics**: Calculate success rates, latency, and similarity scores
- **Detailed Output**: View comprehensive validation results with metadata
- **Error Detection**: Identify empty or irrelevant responses

## Prerequisites

- Python 3.11 or higher
- Access to Qdrant vector database with pre-loaded embeddings
- Cohere API key for embedding generation
- Required Python packages: qdrant-client, cohere, python-dotenv

## Setup

1. Install dependencies:
```bash
pip install -r requirements.txt
```

2. Set up environment variables:
Create a `.env` file with:
```
QDRANT_URL=your_qdrant_url
QDRANT_API_KEY=your_qdrant_api_key
COHERE_API_KEY=your_cohere_api_key
QDRANT_COLLECTION_NAME=book_chunks
```

## Usage

Run the validation script:

```bash
cd backend
python retrieve.py --query "your natural language query here"
```

Or with specific parameters:

```bash
python retrieve.py --query "what is the main topic of the book?" --top-k 5 --collection-name "book_chunks"
```

For batch queries, create a file with queries (one per line) and use:
```bash
python retrieve.py --query @queries.txt --output-format detailed
```

### Options

- `--query`: The natural language query to validate (required)
- `--top-k`: Number of top results to retrieve (default: 5, max: 20)
- `--collection-name`: Name of the Qdrant collection to search
- `--output-format`: Output format (json, text, detailed; default: json)
- `--verbose`: Enable verbose logging output

## Output Format

### Success Response
```json
{
  "query": {
    "text": "the input query string",
    "timestamp": "2023-01-01T00:00:00"
  },
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
  "query": {
    "text": "the input query string",
    "timestamp": "2023-01-01T00:00:00"
  },
  "success": false,
  "error_message": "descriptive error message",
  "execution_time": 0.010
}
```

## Architecture

The validation tool is organized into several modules:

- `retrieve.py`: Main entry point with core validation logic
- `config.py`: Configuration and environment variable management
- `models.py`: Data models for queries, results, and metrics
- `utils/qdrant_client.py`: Qdrant client utility functions
- `cli.py`: Command-line argument parsing
- `logging_util.py`: Logging configuration and utilities
- `error_handling.py`: Error handling and validation utilities