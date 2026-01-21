# Quickstart Guide: RAG Backend-Frontend Integration

## Prerequisites

- Python 3.11+
- Node.js 16+ (for Docusaurus frontend)
- Access to OpenAI API key
- Access to Qdrant vector database (cloud or local)
- Git

## Setup Instructions

### 1. Clone and Navigate to Project

```bash
git clone <repository-url>
cd <project-root>
```

### 2. Set Up Backend Environment

```bash
# Create Python virtual environment
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate

# Install backend dependencies
pip install fastapi uvicorn python-dotenv openai qdrant-client pydantic
```

### 3. Configure Environment Variables

Create a `.env` file in the project root with the following variables:

```env
OPENAI_API_KEY=your_openai_api_key_here
QDRANT_URL=your_qdrant_url_here
QDRANT_API_KEY=your_qdrant_api_key_here  # if using cloud
QDRANT_COLLECTION_NAME=your_collection_name
DEBUG=true  # set to false for production
LOG_LEVEL=info
```

### 4. Start Backend Service

```bash
# Navigate to backend directory
cd backend

# Start the FastAPI server
uvicorn api:app --host 0.0.0.0 --port 8000 --reload
```

The backend will be available at `http://localhost:8000`.

### 5. Set Up Frontend (Docusaurus)

```bash
# From project root
npm install

# Start the Docusaurus development server
npm start
```

The frontend will be available at `http://localhost:3000`.

## API Usage

### Send a Chat Query

```bash
curl -X POST http://localhost:8000/chat \
  -H "Content-Type: application/json" \
  -d '{
    "query": "What are the key principles of RAG systems?",
    "selected_text": "Retrieval-Augmented Generation combines retrieval and generation...",
    "restrict_to_selection": true,
    "conversation_id": "conv_abc123xyz"
  }'
```

### Expected Response

```json
{
  "success": true,
  "data": {
    "response": "RAG systems combine retrieval mechanisms with generative models...",
    "sources": ["document_001", "document_005"],
    "confidence": 0.85,
    "conversation_id": "conv_abc123xyz",
    "timestamp": "2026-01-15T10:30:00Z"
  },
  "request_id": "req_def456uvw"
}
```

## Frontend Integration

The Chatbot UI component will be available as a full-page interaction. It communicates with the backend via the `/chat` endpoint using JSON HTTP requests.

## Development Tips

### Backend Development
- The API server auto-reloads on code changes
- Check the auto-generated API documentation at `http://localhost:8000/docs`
- Logs are output to the console with the configured log level

### Frontend Development
- Docusaurus hot-reloads on changes
- The Chatbot component can be integrated into any documentation page
- Use the existing Docusaurus styling patterns

## Troubleshooting

### Common Issues

1. **Environment variables not loaded**: Ensure `.env` file is in the project root and restart the server
2. **Connection errors**: Verify OpenAI and Qdrant credentials are correct
3. **CORS errors**: The backend should have CORS configured for localhost:3000

### API Documentation
- Full API documentation is available at `http://localhost:8000/docs` when the server is running
- Interactive testing available at the same location