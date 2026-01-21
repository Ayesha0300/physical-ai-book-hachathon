# Quickstart: RAG Pipeline Validation

## Prerequisites

- Python 3.11 or higher
- Access to Qdrant vector database with pre-loaded embeddings
- Cohere API key for embedding validation
- Required Python packages: qdrant-client, cohere, python-dotenv

## Setup

1. Install dependencies:
```bash
pip install qdrant-client cohere python-dotenv
```

2. Set up environment variables:
Create a `.env` file with:
```
QDRANT_URL=your_qdrant_url
QDRANT_API_KEY=your_qdrant_api_key
COHERE_API_KEY=your_cohere_api_key
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

## Example Output

The script will output:
- Retrieved text chunks with similarity scores
- Metadata (URL, section, chunk id)
- Execution time
- Validation metrics