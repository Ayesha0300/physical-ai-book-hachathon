import os
from config import Config
from openai import OpenAI
from mcp_context7_provider import get_retrieval_context
from validators import validate_response_against_context, filter_response_by_context
from typing import List, Dict, Optional
import logging


class RAGAgent:
    """
    RAG Agent that performs semantic retrieval using MCP Context7 protocol from Qdrant
    and generates grounded answers using OpenAI via OpenRouter.
    """

    def __init__(self, answer_only_mode: bool = False):
        """
        Initialize the RAG Agent with configuration and clients.

        Args:
            answer_only_mode: If True, the agent will only answer based on selected text
        """
        # Use the configuration from Config class
        self.openrouter_api_key = Config.OPENROUTER_API_KEY

        # Fallback to OpenAI API key if OpenRouter is not configured
        if not self.openrouter_api_key:
            self.openrouter_api_key = Config.OPENAI_API_KEY
            if not self.openrouter_api_key:
                raise ValueError("Either OPENROUTER_API_KEY or OPENAI_API_KEY environment variable is required")

            # If using OpenAI API key, use the OpenAI base URL instead
            self.openai_client = OpenAI(
                api_key=self.openrouter_api_key
            )
        else:
            # Initialize OpenAI client with OpenRouter base URL
            self.openai_client = OpenAI(
                api_key=self.openrouter_api_key,
                base_url="https://openrouter.ai/api/v1"
            )

        self.answer_only_mode = answer_only_mode

    def query(self, user_query: str) -> str:
        """
        Process a user query by retrieving relevant context and generating a response.

        Args:
            user_query: The query from the user

        Returns:
            Generated response based on retrieved context
        """
        try:
            # Retrieve relevant context from Qdrant
            context_chunks = self.retrieve_context(user_query)

            if not context_chunks:
                if self.answer_only_mode:
                    return "I cannot answer based on the provided context."
                else:
                    return "No relevant context found for your query."

            # Format context for OpenAI
            context_text = "\n".join([chunk.get('content', '') for chunk in context_chunks if 'content' in chunk])

            # Generate response using OpenAI with the retrieved context
            response = self.generate_response_with_openai(user_query, context_text)

            if response is None:
                return "Error generating response from the language model."

            # Validate response if in answer-only mode
            if self.answer_only_mode:
                if not validate_response_against_context(response, context_chunks):
                    return "I cannot answer based on the provided context."

                # Filter response to ensure compliance
                response = filter_response_by_context(response, context_chunks)

            return response

        except Exception as e:
            logging.error(f"Error processing query: {e}")
            return "An error occurred while processing your query."

    def generate_response_with_openai(self, user_query: str, context_text: str) -> str:
        """
        Generate a response using OpenAI via OpenRouter with the provided context.

        Args:
            user_query: The original user query
            context_text: The retrieved context to provide to the model

        Returns:
            Generated response from OpenAI
        """
        try:
            # Construct the prompt with context
            system_prompt = f"You are a helpful assistant. Use the following context to answer the user's question:\n\n{context_text}\n\nIf the context doesn't contain the information needed to answer the question, please say so."

            # Use the model from configuration, with fallback to default
            if hasattr(Config, 'OPENROUTER_MODEL'):
                model_name = Config.OPENROUTER_MODEL
            elif hasattr(Config, 'OPENAI_MODEL'):
                model_name = Config.OPENAI_MODEL
            else:
                model_name = 'mistralai/mistral-7b-instruct:free'  # fallback default

            response = self.openai_client.chat.completions.create(
                model=model_name,  # Using the model from environment config
                messages=[
                    {"role": "system", "content": system_prompt},
                    {"role": "user", "content": user_query}
                ],
                max_tokens=1000,
                temperature=0.7
            )

            return response.choices[0].message.content.strip()

        except Exception as e:
            logging.error(f"Error generating response with OpenAI: {e}")
            return None

    def retrieve_context(self, query: str) -> List[Dict]:
        """
        Retrieve relevant context chunks from Qdrant using MCP Context7 protocol based on the query.

        Args:
            query: The query to search for relevant context

        Returns:
            List of context chunks retrieved from Qdrant via MCP Context7
        """
        try:
            # Use the MCP Context7 provider to retrieve context
            context_response = get_retrieval_context(query)

            # Extract the chunks from the context response
            chunks_data = context_response.get('metadata', {}).get('chunks', [])

            # Format the results to match the expected structure
            context_chunks = []
            for chunk in chunks_data:
                context_chunks.append({
                    'id': chunk.get('id'),
                    'content': chunk.get('content', ''),
                    'metadata': chunk.get('metadata', {}),
                    'score': chunk.get('score', 0.0)
                })

            return context_chunks

        except Exception as e:
            logging.error(f"Error retrieving context via MCP Context7: {e}")
            return []

    def toggle_answer_only_mode(self, enabled: bool):
        """
        Toggle the 'answer only from selected text' mode.

        Args:
            enabled: True to enable answer-only mode, False to disable
        """
        self.answer_only_mode = enabled

    def check_health(self) -> Dict[str, bool]:
        """
        Check the health of all connected services.

        Returns:
            Dictionary with health status of each service
        """
        # Test OpenAI connection by making a simple API call
        openai_healthy = self._check_openai_connection()

        # Test retrieval functionality by attempting a simple retrieval
        try:
            # Attempt to retrieve context for a simple test query
            test_context = self.retrieve_context("test")
            retrieval_healthy = test_context is not None
        except:
            retrieval_healthy = False

        return {
            'openai': openai_healthy,
            'retrieval': retrieval_healthy,
            'overall': openai_healthy and retrieval_healthy
        }

    def _check_openai_connection(self) -> bool:
        """
        Check if the OpenAI API connection is working.

        Returns:
            Boolean indicating if the connection is healthy
        """
        try:
            # Test with a simple model listing request
            self.openai_client.models.list()
            return True
        except Exception:
            return False