"""
Pydantic models for conversation management in the RAG Chatbot backend
"""

from pydantic import BaseModel, Field
from typing import Optional, List
from datetime import datetime


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


class Message(BaseModel):
    """
    Model for individual messages within a conversation
    """
    id: str = Field(..., description="Unique identifier for the message")
    role: str = Field(..., description="Role of the message sender (user/assistant)")
    content: str = Field(..., min_length=1, description="Content of the message")
    timestamp: datetime = Field(default_factory=datetime.utcnow, description="When the message was created")
    metadata: Optional[dict] = Field(None, description="Additional metadata about the message")