from pydantic import BaseModel, Field
from typing import Optional, List
from datetime import datetime
from enum import Enum
import uuid


class SenderType(str, Enum):
    """
    Enum for message sender types
    """
    USER = "user"
    BOT = "bot"
    SYSTEM = "system"


class ChatMessage(BaseModel):
    """
    Data model representing a single message in the conversation
    """
    id: str = Field(default_factory=lambda: str(uuid.uuid4()), description="Unique identifier for the message")
    text: str = Field(
        ...,
        min_length=1,
        max_length=4000,
        description="The content of the message"
    )
    sender: SenderType = Field(
        ...,
        description="The type of sender (user/bot/system)"
    )
    timestamp: str = Field(
        default_factory=lambda: datetime.utcnow().isoformat(),
        description="The time when the message was created/sent"
    )
    sources: Optional[List[str]] = Field(
        default=None,
        max_items=10,
        description="Array of source references for bot responses"
    )
    confidence: Optional[float] = Field(
        default=None,
        ge=0.0,
        le=1.0,
        description="Confidence level for bot responses (0-1)"
    )

    class Config:
        schema_extra = {
            "example": {
                "id": "msg_12345",
                "text": "What is reinforcement learning?",
                "sender": "user",
                "timestamp": "2026-01-21T10:30:00Z",
            }
        }


class ChatSessionState(str, Enum):
    """
    Enum for chat session states
    """
    ACTIVE = "active"
    INACTIVE = "inactive"
    EXPIRED = "expired"


class ChatSession(BaseModel):
    """
    Data model representing a conversation context that may persist across page navigations
    """
    id: str = Field(
        default_factory=lambda: str(uuid.uuid4()),
        description="Unique identifier for the session"
    )
    created_at: str = Field(
        default_factory=lambda: datetime.utcnow().isoformat(),
        description="The time when the session was created"
    )
    last_active_at: str = Field(
        default_factory=lambda: datetime.utcnow().isoformat(),
        description="The time when the session was last active"
    )
    is_active: bool = Field(
        default=True,
        description="Whether the session is currently active"
    )
    state: ChatSessionState = Field(
        default=ChatSessionState.ACTIVE,
        description="Current state of the session"
    )

    class Config:
        schema_extra = {
            "example": {
                "id": "sess_12345",
                "created_at": "2026-01-21T10:30:00Z",
                "last_active_at": "2026-01-21T10:35:00Z",
                "is_active": True,
                "state": "active"
            }
        }