"""
MCP (Model Context Protocol) Server for the RAG Agent with OpenAI Integration
Provides a standardized interface for model context providers to interact with the RAG system.
"""

import json
import logging
from typing import Dict, Any, List, Optional
from fastapi import FastAPI, HTTPException
from pydantic import BaseModel
import uvicorn

from agent import RAGAgent
from config import Config

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

app = FastAPI(title="Context7 RAG Agent MCP Server", version="1.0.0")

# Global agent instance
rag_agent: Optional[RAGAgent] = None


class QueryRequest(BaseModel):
    """Request model for querying the RAG agent."""
    query: str
    context_filter: Optional[Dict[str, Any]] = None


class QueryResponse(BaseModel):
    """Response model for RAG agent queries."""
    response: str
    context_used: List[Dict[str, Any]]
    metadata: Dict[str, Any]


@app.on_event("startup")
async def startup_event():
    """Initialize the RAG agent when the server starts."""
    global rag_agent
    try:
        rag_agent = RAGAgent(answer_only_mode=False)
        logger.info("RAG Agent initialized successfully")
    except Exception as e:
        logger.error(f"Failed to initialize RAG Agent: {e}")
        raise


@app.post("/query", response_model=QueryResponse)
async def query_endpoint(request: QueryRequest):
    """Endpoint to process queries through the RAG agent."""
    global rag_agent

    if rag_agent is None:
        raise HTTPException(status_code=500, detail="RAG Agent not initialized")

    try:
        # Process the query through the RAG agent
        response = rag_agent.query(request.query)

        # For now, we'll return an empty context list since the retrieve_context method is not fully implemented
        # In a complete implementation, this would return the actual context used
        context_used = []

        return QueryResponse(
            response=response,
            context_used=context_used,
            metadata={
                "model": Config.OPENAI_MODEL,
                "provider": "openai"
            }
        )
    except Exception as e:
        logger.error(f"Error processing query: {e}")
        raise HTTPException(status_code=500, detail=f"Error processing query: {str(e)}")


@app.get("/health")
async def health_check():
    """Health check endpoint."""
    global rag_agent

    if rag_agent is None:
        return {"status": "error", "details": "RAG Agent not initialized"}

    try:
        health_status = rag_agent.check_health()
        return {"status": "ok", "services": health_status}
    except Exception as e:
        logger.error(f"Health check failed: {e}")
        return {"status": "error", "details": str(e)}


@app.get("/capabilities")
async def capabilities():
    """Return the capabilities of this MCP server."""
    return {
        "name": "Context7 RAG Agent",
        "version": "1.0.0",
        "capabilities": [
            "contextual-query-processing",
            "vector-search-integration",
            "openai-integration",
            "health-monitoring"
        ],
        "models_supported": [Config.OPENAI_MODEL],
        "integrations": ["qdrant", "openai"]
    }


@app.post("/validate-context")
async def validate_context(context: Dict[str, Any]):
    """Validate if the provided context is usable by the agent."""
    global rag_agent

    if rag_agent is None:
        raise HTTPException(status_code=500, detail="RAG Agent not initialized")

    # In a real implementation, this would validate the context
    # For now, we just return a basic validation result
    is_valid = "content" in context or "chunks" in context
    return {"valid": is_valid, "message": "Context validated" if is_valid else "Invalid context format"}


def main():
    """Run the MCP server."""
    logger.info("Starting Context7 RAG Agent MCP Server...")

    # Verify configuration before starting
    try:
        Config.validate()
        logger.info("Configuration validated successfully")
    except ValueError as e:
        logger.error(f"Configuration validation failed: {e}")
        return

    uvicorn.run(
        app,
        host="0.0.0.0",
        port=8001,
        log_level="info"
    )


if __name__ == "__main__":
    main()