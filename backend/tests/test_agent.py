"""
Test suite for the RAG Agent
"""

import pytest
from unittest.mock import patch, MagicMock
import os

from agent import RAGAgent


def test_rag_agent_initialization():
    """Test RAG Agent initialization"""
    # Mock environment variable
    with patch.dict(os.environ, {'OPENROUTER_API_KEY': 'test-key'}):
        agent = RAGAgent()

        assert agent is not None
        assert hasattr(agent, 'openai_client')
        assert hasattr(agent, 'answer_only_mode')


def test_rag_agent_initialization_missing_api_key():
    """Test RAG Agent initialization without API key raises error"""
    # Ensure OPENROUTER_API_KEY is not in environment
    with patch.dict(os.environ, {}, clear=True):
        with pytest.raises(ValueError, match="OPENROUTER_API_KEY environment variable is required"):
            RAGAgent()


@patch('agent.OpenAI')
def test_rag_agent_query(mock_openai_class):
    """Test RAG Agent query method"""
    # Mock environment variable
    with patch.dict(os.environ, {'OPENROUTER_API_KEY': 'test-key'}):
        agent = RAGAgent()

        # Mock the client and response
        mock_client_instance = MagicMock()
        mock_response = MagicMock()
        mock_response.choices = [MagicMock()]
        mock_response.choices[0].message.content = "Test response"
        mock_client_instance.chat.completions.create.return_value = mock_response
        agent.openai_client = mock_client_instance

        # Mock the retrieve_context method to return some context
        with patch.object(agent, 'retrieve_context') as mock_retrieve:
            mock_retrieve.return_value = [
                {
                    'id': 'test_id',
                    'content': 'Test context content',
                    'metadata': {},
                    'score': 0.9
                }
            ]

            result = agent.query("Test question?")

            assert result == "Test response"
            mock_retrieve.assert_called_once_with("Test question?")


@patch('agent.OpenAI')
def test_rag_agent_query_no_context(mock_openai_class):
    """Test RAG Agent query method when no context is found"""
    # Mock environment variable
    with patch.dict(os.environ, {'OPENROUTER_API_KEY': 'test-key'}):
        agent = RAGAgent(answer_only_mode=True)

        # Mock the retrieve_context method to return empty context
        with patch.object(agent, 'retrieve_context') as mock_retrieve:
            mock_retrieve.return_value = []

            result = agent.query("Test question?")

            assert result == "I cannot answer based on the provided context."


@patch('agent.OpenAI')
def test_rag_agent_health_check(mock_openai_class):
    """Test RAG Agent health check"""
    # Mock environment variable
    with patch.dict(os.environ, {'OPENROUTER_API_KEY': 'test-key'}):
        agent = RAGAgent()

        # Mock the model listing to succeed
        mock_models = MagicMock()
        mock_models.data = [{'id': 'test-model'}]
        agent.openai_client.models.list.return_value = mock_models

        # Mock retrieve_context to succeed
        with patch.object(agent, 'retrieve_context') as mock_retrieve:
            mock_retrieve.return_value = [{'content': 'test'}]

            health = agent.check_health()

            assert health['overall'] is True
            assert health['openai'] is True
            assert health['retrieval'] is True


if __name__ == "__main__":
    pytest.main([__file__])