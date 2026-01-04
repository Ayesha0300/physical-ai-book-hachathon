# Quickstart: RAG Content Ingestion Pipeline

**Feature**: RAG Content Ingestion Pipeline
**Date**: 2026-01-04
**Branch**: 001-rag-pipeline

## Overview

This guide provides step-by-step instructions to set up and run the RAG content ingestion pipeline that crawls website content, chunks it, generates embeddings, and stores them in Qdrant Cloud.

## Prerequisites

- Python 3.11 or higher
- pip package manager
- Access to Cohere API (API key)
- Access to Qdrant Cloud (URL and API key)
- Git for version control

## Setup Instructions

### 1. Clone the Repository
```bash
git clone <repository-url>
cd <repository-name>
git checkout 001-rag-pipeline
```

### 2. Navigate to Backend Directory
```bash
cd backend/
```

### 3. Create Virtual Environment
```bash
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
```

### 4. Initialize the Project
```bash
uv init
```

### 5. Install Dependencies
The following dependencies will be added to your project:

```bash
pip install requests beautifulsoup4 cohere qdrant-client python-dotenv tqdm
```

### 6. Configure Environment Variables
Create a `.env` file in the backend directory with the following content:

```env
COHERE_API_KEY=your_cohere_api_key_here
QDRANT_URL=your_qdrant_cloud_url_here
QDRANT_API_KEY=your_qdrant_api_key_here
QDRANT_COLLECTION_NAME=rag_content_chunks
CHUNK_SIZE=512
CHUNK_OVERLAP=50
REQUEST_TIMEOUT=30
BATCH_SIZE=10
SITEMAP_URL=https://physical-ai-book-hachathon.vercel.app/sitemap.xml
```

## Running the Pipeline

### 1. Execute the Ingestion Pipeline
```bash
python main.py
```

### 2. Monitor the Process
The pipeline will:
1. Fetch the sitemap from the configured URL
2. Extract content from each URL in the sitemap
3. Chunk the content according to the configured parameters
4. Generate embeddings using Cohere
5. Store the embeddings in Qdrant Cloud
6. Provide progress updates throughout the process

### 3. Verify the Results
After completion, you can verify that vectors were successfully stored by checking the collection in Qdrant Cloud or running a sample similarity query.

## Configuration Options

### Environment Variables
- `COHERE_API_KEY`: Your Cohere API key for embedding generation
- `QDRANT_URL`: URL for your Qdrant Cloud instance
- `QDRANT_API_KEY`: API key for your Qdrant Cloud instance
- `QDRANT_COLLECTION_NAME`: Name of the collection to store embeddings (default: rag_content_chunks)
- `CHUNK_SIZE`: Maximum size of content chunks in characters (default: 512)
- `CHUNK_OVERLAP`: Overlap between chunks in characters (default: 50)
- `REQUEST_TIMEOUT`: Timeout for HTTP requests in seconds (default: 30)
- `BATCH_SIZE`: Number of items to process in each batch (default: 10)
- `SITEMAP_URL`: URL of the sitemap.xml to process

## Development Workflow

### 1. Code Structure
The entire pipeline is contained in a single `main.py` file with the following structure:
- `main()` function orchestrates the entire workflow
- Individual functions for each step of the pipeline
- Configuration loading from environment variables
- Error handling and logging throughout

### 2. Testing the Pipeline
To test the pipeline with a small subset of URLs:
1. Modify the `SITEMAP_URL` in your `.env` file to point to a test sitemap
2. Reduce the `CHUNK_SIZE` for faster processing during testing
3. Run the pipeline and verify output

### 3. Troubleshooting
Common issues and solutions:
- **API rate limits**: Reduce the number of concurrent requests or add delays between API calls
- **Memory issues**: Process content in smaller batches
- **Network timeouts**: Increase the `REQUEST_TIMEOUT` value in your `.env` file
- **Invalid content**: The pipeline should handle malformed HTML gracefully

## Verification Steps

### 1. Check Vector Count
After running the pipeline, verify the number of vectors stored in Qdrant matches expectations.

### 2. Sample Query Test
Run a sample similarity query to ensure vectors are retrievable and semantically meaningful.

### 3. Log Review
Review the structured logs to ensure all steps completed successfully and no errors occurred.