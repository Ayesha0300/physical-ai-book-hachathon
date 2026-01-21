"""
Conversation Service for the RAG Chatbot backend
Manages conversation state and context for multi-turn interactions
"""

import asyncio
import logging
from typing import List, Dict, Optional
from datetime import datetime
from uuid import uuid4

from models.response import ConversationContext, ChatResponse
from config import Config


class ConversationService:
    """
    Service for managing conversation state and context
    """

    def __init__(self):
        """
        Initialize the conversation service
        In a real implementation, this would connect to a database or cache
        For this implementation, we'll use an in-memory store
        """
        self.conversations: Dict[str, ConversationContext] = {}
        logging.info("Conversation service initialized")

    async def create_conversation(self, conversation_id: Optional[str] = None) -> ConversationContext:
        """
        Create a new conversation

        Args:
            conversation_id: Optional ID for the conversation (will generate if not provided)

        Returns:
            New ConversationContext object
        """
        try:
            conv_id = conversation_id or str(uuid4())

            new_conversation = ConversationContext(
                conversation_id=conv_id,
                messages=[],
                created_at=datetime.utcnow(),
                updated_at=datetime.utcnow(),
                max_messages=Config.MAX_MESSAGES_PER_CONVERSATION if hasattr(Config, 'MAX_MESSAGES_PER_CONVERSATION') else 10
            )

            self.conversations[conv_id] = new_conversation

            logging.info(f"Created new conversation: {conv_id}")

            return new_conversation

        except Exception as e:
            logging.error(f"Error creating conversation: {str(e)}")
            raise

    async def get_conversation(self, conversation_id: str) -> Optional[ConversationContext]:
        """
        Get an existing conversation

        Args:
            conversation_id: ID of the conversation to retrieve

        Returns:
            ConversationContext object or None if not found
        """
        try:
            conversation = self.conversations.get(conversation_id)

            if conversation:
                logging.debug(f"Retrieved conversation: {conversation_id}")
            else:
                logging.info(f"Conversation not found: {conversation_id}")

            return conversation

        except Exception as e:
            logging.error(f"Error getting conversation {conversation_id}: {str(e)}")
            return None

    async def add_message_to_conversation(
        self,
        conversation_id: str,
        role: str,  # 'user' or 'assistant'
        content: str,
        metadata: Optional[Dict] = None
    ) -> bool:
        """
        Add a message to a conversation

        Args:
            conversation_id: ID of the conversation
            role: Role of the message sender ('user' or 'assistant')
            content: Content of the message
            metadata: Optional metadata about the message

        Returns:
            Boolean indicating success
        """
        try:
            conversation = await self.get_conversation(conversation_id)

            if not conversation:
                logging.warning(f"Cannot add message to non-existent conversation: {conversation_id}")
                return False

            # Create message object
            message = {
                "id": str(uuid4()),
                "role": role,
                "content": content,
                "timestamp": datetime.utcnow(),
                "metadata": metadata or {}
            }

            # Add message to conversation
            conversation.messages.append(message)

            # Update the updated_at timestamp
            conversation.updated_at = datetime.utcnow()

            # Trim conversation if it exceeds max_messages
            max_messages = getattr(Config, 'MAX_MESSAGES_PER_CONVERSATION', 10)
            if len(conversation.messages) > max_messages:
                # Keep the most recent messages, removing older ones
                conversation.messages = conversation.messages[-max_messages:]

            logging.debug(f"Added message to conversation {conversation_id}, now has {len(conversation.messages)} messages")

            return True

        except Exception as e:
            logging.error(f"Error adding message to conversation {conversation_id}: {str(e)}")
            return False

    async def get_conversation_history(
        self,
        conversation_id: str,
        limit: Optional[int] = None
    ) -> List[Dict]:
        """
        Get the history of messages in a conversation

        Args:
            conversation_id: ID of the conversation
            limit: Optional limit on number of messages to return (gets most recent)

        Returns:
            List of message dictionaries
        """
        try:
            conversation = await self.get_conversation(conversation_id)

            if not conversation:
                logging.warning(f"Cannot get history for non-existent conversation: {conversation_id}")
                return []

            messages = conversation.messages

            # Apply limit if specified
            if limit:
                messages = messages[-limit:]

            logging.debug(f"Retrieved {len(messages)} messages for conversation {conversation_id}")

            return messages

        except Exception as e:
            logging.error(f"Error getting conversation history {conversation_id}: {str(e)}")
            return []

    async def update_conversation_context(
        self,
        conversation_id: str,
        context_data: Dict
    ) -> bool:
        """
        Update conversation context with additional data

        Args:
            conversation_id: ID of the conversation
            context_data: Dictionary with context data to update

        Returns:
            Boolean indicating success
        """
        try:
            conversation = await self.get_conversation(conversation_id)

            if not conversation:
                logging.warning(f"Cannot update context for non-existent conversation: {conversation_id}")
                return False

            # For this implementation, we're not storing additional context beyond messages
            # In a more complex implementation, this could update additional fields

            # Update the updated_at timestamp
            conversation.updated_at = datetime.utcnow()

            logging.debug(f"Updated context for conversation {conversation_id}")

            return True

        except Exception as e:
            logging.error(f"Error updating conversation context {conversation_id}: {str(e)}")
            return False

    async def clear_conversation(self, conversation_id: str) -> bool:
        """
        Clear all messages from a conversation (keeps the conversation object)

        Args:
            conversation_id: ID of the conversation to clear

        Returns:
            Boolean indicating success
        """
        try:
            conversation = await self.get_conversation(conversation_id)

            if not conversation:
                logging.warning(f"Cannot clear non-existent conversation: {conversation_id}")
                return False

            # Clear the messages but keep the conversation object
            conversation.messages = []
            conversation.updated_at = datetime.utcnow()

            logging.info(f"Cleared messages from conversation {conversation_id}")

            return True

        except Exception as e:
            logging.error(f"Error clearing conversation {conversation_id}: {str(e)}")
            return False

    async def delete_conversation(self, conversation_id: str) -> bool:
        """
        Delete a conversation completely

        Args:
            conversation_id: ID of the conversation to delete

        Returns:
            Boolean indicating success
        """
        try:
            if conversation_id in self.conversations:
                del self.conversations[conversation_id]

                logging.info(f"Deleted conversation: {conversation_id}")

                return True
            else:
                logging.warning(f"Cannot delete non-existent conversation: {conversation_id}")

                return False

        except Exception as e:
            logging.error(f"Error deleting conversation {conversation_id}: {str(e)}")
            return False

    async def get_active_conversations(self) -> List[str]:
        """
        Get list of active conversation IDs

        Returns:
            List of conversation IDs
        """
        try:
            # In this in-memory implementation, all conversations are "active"
            # In a real implementation, you might want to filter by last activity time
            active_conv_ids = list(self.conversations.keys())

            logging.debug(f"Found {len(active_conv_ids)} active conversations")

            return active_conv_ids

        except Exception as e:
            logging.error(f"Error getting active conversations: {str(e)}")
            return []


# Global conversation service instance
conversation_service = ConversationService()


# Async wrappers for use in endpoints
async def create_new_conversation(conversation_id: Optional[str] = None) -> ConversationContext:
    """Create a new conversation"""
    return await conversation_service.create_conversation(conversation_id)


async def get_conversation_by_id(conversation_id: str) -> Optional[ConversationContext]:
    """Get an existing conversation by ID"""
    return await conversation_service.get_conversation(conversation_id)


async def add_message_to_existing_conversation(
    conversation_id: str,
    role: str,
    content: str,
    metadata: Optional[Dict] = None
) -> bool:
    """Add a message to an existing conversation"""
    return await conversation_service.add_message_to_conversation(
        conversation_id, role, content, metadata
    )


async def get_conversation_history_by_id(
    conversation_id: str,
    limit: Optional[int] = None
) -> List[Dict]:
    """Get conversation history by ID"""
    return await conversation_service.get_conversation_history(conversation_id, limit)