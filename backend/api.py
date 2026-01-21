"""
API router for the RAG Chatbot backend
Defines the /chat endpoint and related functionality
"""

from fastapi import APIRouter, HTTPException, Depends
from typing import Optional
import uuid
import logging
from datetime import datetime

from models.request import ChatRequest, ChatSuccessResponse, ChatErrorResponse
from models.response import ChatResponse
from agent import RAGAgent
from config import Config

# Create API router
router = APIRouter()

# Initialize the RAG agent
rag_agent = RAGAgent()

@router.post("/chat", response_model=ChatSuccessResponse)
async def chat_endpoint(chat_request: ChatRequest):
    """
    Process a user query and return an AI-generated response
    Implements the /chat endpoint as specified in the API contract
    """
    request_id = str(uuid.uuid4())

    try:
        # Log the incoming request
        logging.info(f"Processing chat request {request_id}: {chat_request.query[:100]}...")

        # Validate the request
        if not chat_request.query or not chat_request.query.strip():
            error_response = ChatErrorResponse(
                error={
                    "code": "INVALID_QUERY",
                    "message": "Query must be provided and cannot be empty",
                    "details": {
                        "field": "query",
                        "reason": "Field is required and cannot be empty"
                    }
                },
                request_id=request_id
            )
            raise HTTPException(status_code=400, detail=error_response.error)

        # Process the query through the RAG agent
        # For now, we'll call the existing agent.query method, but in a full implementation
        # this would include conversation context
        response_text = rag_agent.query(chat_request.query)

        # Generate conversation ID if not provided
        conversation_id = chat_request.conversation_id or str(uuid.uuid4())

        # Create the response object
        chat_response_data = ChatResponse(
            response=response_text,
            sources=[],  # Sources would be populated by the agent
            confidence=0.8,  # Default confidence
            conversation_id=conversation_id,
            timestamp=datetime.utcnow()
        )

        # Prepare the success response
        success_response = ChatSuccessResponse(
            data=chat_response_data.dict(),
            request_id=request_id
        )

        # Log successful response
        logging.info(f"Successfully processed chat request {request_id}")

        return success_response

    except HTTPException:
        # Re-raise HTTP exceptions as-is
        raise
    except Exception as e:
        # Handle any unexpected errors
        logging.error(f"Error processing chat request {request_id}: {str(e)}")

        error_response = ChatErrorResponse(
            error={
                "code": "INTERNAL_ERROR",
                "message": "An internal error occurred while processing your request",
                "details": {
                    "error_type": type(e).__name__,
                    "error_message": str(e)
                }
            },
            request_id=request_id
        )

        raise HTTPException(status_code=500, detail=error_response.error)


@router.get("/health")
async def api_health():
    """
    Health check endpoint for the API
    """
    return {
        "status": "healthy",
        "endpoint": "/chat",
        "configured": True
    }


# Additional endpoints can be added here as needed