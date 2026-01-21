from pydantic import BaseModel, Field
from typing import Optional, List
from datetime import datetime


class ChatResponse(BaseModel):
    """
    Response model for chat endpoint
    """
    response: str = Field(
        ...,
        description="The chatbot's response to the user's query"
    )
    sources: Optional[List[str]] = Field(
        default=None,
        max_items=10,
        description="List of sources used to generate the response"
    )
    confidence: Optional[float] = Field(
        default=None,
        ge=0.0,
        le=1.0,
        description="Confidence level of the response (0-1)"
    )
    conversation_id: str = Field(
        ...,
        description="ID of the conversation"
    )
    timestamp: str = Field(
        ...,
        description="Timestamp when the response was generated"
    )


class ChatSuccessResponse(BaseModel):
    """
    Success response wrapper for chat endpoint
    """
    success: bool = True
    data: ChatResponse
    request_id: str = Field(
        ...,
        description="Unique ID for the request for tracking purposes"
    )


class ChatErrorResponse(BaseModel):
    """
    Error response model for chat endpoint
    """
    success: bool = False
    error: dict = Field(
        ...,
        description="Error details including code, message, and additional info"
    )
    request_id: str = Field(
        ...,
        description="Unique ID for the request for tracking purposes"
    )