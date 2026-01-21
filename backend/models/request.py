"""
Pydantic models for request validation in the RAG Chatbot backend
"""

from pydantic import BaseModel, Field, validator
from typing import Optional, List
import uuid
from datetime import datetime


class ChatRequest(BaseModel):
    """
    Model for chat requests to the RAG system
    Extends the ChatQuery entity from the data model
    """
    query: str = Field(..., min_length=1, max_length=2000, description="The user's question or request")
    selected_text: Optional[str] = Field(None, max_length=10000, description="Specific text selected by the user for context")
    restrict_to_selection: bool = Field(False, description="Whether to limit responses to selected text")
    conversation_id: Optional[str] = Field(None, description="ID for maintaining conversation context")

    @validator('query')
    def validate_query(cls, v):
        if not v or v.strip() == "":
            raise ValueError('Query cannot be empty or whitespace only')
        return v.strip()


class ChatSuccessResponse(BaseModel):
    """
    Model for successful chat responses
    Follows the API contract specification
    """
    success: bool = True
    data: dict  # Will contain ChatResponse data
    request_id: str = Field(default_factory=lambda: str(uuid.uuid4()))


class ChatErrorResponse(BaseModel):
    """
    Model for error responses
    Follows the API contract specification
    """
    success: bool = False
    error: dict
    request_id: str = Field(default_factory=lambda: str(uuid.uuid4()))