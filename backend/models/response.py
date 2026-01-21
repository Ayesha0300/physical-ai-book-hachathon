"""
Pydantic models for response validation in the RAG Chatbot backend
"""

from pydantic import BaseModel, Field
from typing import Optional, List
from datetime import datetime


class ChatResponse(BaseModel):
    """
    Model for chat responses from the RAG system
    Extends the ChatResponse entity from the data model
    """
    response: str = Field(..., min_length=1, max_length=10000, description="The AI-generated answer")
    sources: Optional[List[str]] = Field(default=[], max_items=20, description="List of document sources used")
    confidence: Optional[float] = Field(None, ge=0, le=1, description="Confidence score of the response")
    conversation_id: str = Field(..., description="ID for conversation continuity")
    timestamp: datetime = Field(default_factory=datetime.utcnow, description="When the response was generated")
    error: Optional[dict] = Field(None, description="Error information if request failed")


class VectorSearchResult(BaseModel):
    """
    Model for vector search results from the database
    Corresponds to the VectorSearchResult entity from the data model
    """
    content: str = Field(..., min_length=1, max_length=5000, description="The retrieved text content")
    score: float = Field(..., ge=0, le=1, description="Similarity score 0-1")
    metadata: Optional[dict] = Field(None, description="Additional document metadata")
    document_id: str = Field(..., description="Unique identifier for the document chunk")


class ConversationContext(BaseModel):
    """
    Model for maintaining conversation state
    Corresponds to the ConversationContext entity from the data model
    """
    conversation_id: str = Field(..., description="Unique identifier for the conversation")
    messages: List[dict] = Field(..., min_items=1, max_items=50, description="History of messages in the conversation")
    created_at: datetime = Field(default_factory=datetime.utcnow, description="When conversation started")
    updated_at: datetime = Field(default_factory=datetime.utcnow, description="When last updated")
    max_messages: Optional[int] = Field(10, ge=1, le=100, description="Maximum messages to retain")