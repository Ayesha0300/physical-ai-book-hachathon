from pydantic import BaseModel, Field
from typing import Optional
import uuid


class ChatRequest(BaseModel):
    """
    Request model for chat endpoint
    """
    query: str = Field(
        ...,
        min_length=1,
        max_length=2000,
        description="The user's query or question"
    )
    selected_text: Optional[str] = Field(
        default=None,
        max_length=5000,
        description="Optional text selected by the user to provide context"
    )
    restrict_to_selection: bool = Field(
        default=False,
        description="Whether to limit the response to the selected text only"
    )
    conversation_id: Optional[str] = Field(
        default=None,
        regex=r'^[a-zA-Z0-9_-]+$',
        description="Optional ID to maintain conversation context"
    )

    class Config:
        schema_extra = {
            "example": {
                "query": "Explain the concept of reinforcement learning",
                "selected_text": "Reinforcement learning is a type of machine learning where an agent learns to make decisions by performing actions in an environment.",
                "restrict_to_selection": False,
                "conversation_id": "conv_12345"
            }
        }