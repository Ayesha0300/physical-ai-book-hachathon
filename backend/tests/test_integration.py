"""
Integration tests for the RAG Chatbot system
Tests the integration between components
"""

import pytest
from fastapi.testclient import TestClient
from unittest.mock import patch, MagicMock
import json

from main import app
from api import rag_agent
from models.request import ChatRequest


def test_full_chat_flow():
    """Test the complete chat flow from API to agent to response"""
    with TestClient(app) as client:
        # Mock the agent response
        with patch('api.rag_agent') as mock_agent:
            mock_agent.query.return_value = "This is a comprehensive test response based on the context."

            # Mock the retrieve_context method to return some context
            with patch.object(mock_agent, 'retrieve_context') as mock_retrieve:
                mock_retrieve.return_value = [
                    {
                        'id': 'doc_123',
                        'content': 'This is relevant context from the document.',
                        'metadata': {'source': 'test_doc.pdf', 'page': 5},
                        'score': 0.85
                    }
                ]

                response = client.post(
                    "/api/v1/chat",
                    json={
                        "query": "What is the main concept discussed in the document?",
                        "selected_text": None,
                        "restrict_to_selection": False,
                        "conversation_id": "test_conv_789"
                    }
                )

                assert response.status_code == 200
                data = response.json()

                assert data["success"] is True
                assert "data" in data
                assert "response" in data["data"]
                assert "conversation_id" in data["data"]
                assert "timestamp" in data["data"]

                # Verify the response contains the expected content
                assert "comprehensive test response" in data["data"]["response"].lower()


def test_chat_flow_with_selected_text():
    """Test the chat flow with selected text functionality"""
    with TestClient(app) as client:
        # Mock the agent response
        with patch('api.rag_agent') as mock_agent:
            mock_agent.query.return_value = "Based on the selected text, the answer is specific to that context."

            # Mock the retrieve_context method
            with patch.object(mock_agent, 'retrieve_context') as mock_retrieve:
                mock_retrieve.return_value = [
                    {
                        'id': 'doc_456',
                        'content': 'Selected text: This is the specific content the user highlighted.',
                        'metadata': {'source': 'important_section.md', 'page': 12},
                        'score': 0.92
                    }
                ]

                response = client.post(
                    "/api/v1/chat",
                    json={
                        "query": "Explain this specific part?",
                        "selected_text": "This is the specific content the user highlighted.",
                        "restrict_to_selection": True,
                        "conversation_id": "test_conv_001"
                    }
                )

                assert response.status_code == 200
                data = response.json()

                assert data["success"] is True
                assert "specific part" in data["data"]["response"].lower()


def test_conversation_context_preservation():
    """Test that conversation context is preserved between requests"""
    with TestClient(app) as client:
        # First request - establish conversation
        with patch('api.rag_agent') as mock_agent:
            mock_agent.query.return_value = "This is the first response in our conversation."

            with patch.object(mock_agent, 'retrieve_context') as mock_retrieve:
                mock_retrieve.return_value = [{'content': 'test context'}]

                first_response = client.post(
                    "/api/v1/chat",
                    json={
                        "query": "Hello, I'd like to know about the topic.",
                        "selected_text": None,
                        "restrict_to_selection": False
                    }
                )

                assert first_response.status_code == 200
                first_data = first_response.json()

                assert first_data["success"] is True
                conversation_id = first_data["data"]["conversation_id"]
                assert conversation_id is not None

        # Second request - continue conversation
        with patch('api.rag_agent') as mock_agent:
            mock_agent.query.return_value = "This is the follow-up response, continuing our conversation."

            with patch.object(mock_agent, 'retrieve_context') as mock_retrieve:
                mock_retrieve.return_value = [{'content': 'test context'}]

                second_response = client.post(
                    "/api/v1/chat",
                    json={
                        "query": "Can you elaborate on that?",
                        "selected_text": None,
                        "restrict_to_selection": False,
                        "conversation_id": conversation_id
                    }
                )

                assert second_response.status_code == 200
                second_data = second_response.json()

                assert second_data["success"] is True
                assert second_data["data"]["conversation_id"] == conversation_id


def test_error_handling_integration():
    """Test error handling across the full stack"""
    with TestClient(app) as client:
        # Test agent error propagation
        with patch('api.rag_agent') as mock_agent:
            mock_agent.query.side_effect = Exception("Agent error")

            response = client.post(
                "/api/v1/chat",
                json={
                    "query": "This should trigger an error.",
                    "selected_text": None,
                    "restrict_to_selection": False
                }
            )

            assert response.status_code == 500
            error_data = response.json()

            assert error_data["success"] is False
            assert "INTERNAL_ERROR" in error_data["error"]["code"]


def test_health_endpoints_integration():
    """Test that all health endpoints work together"""
    with TestClient(app) as client:
        # Test main health endpoint
        health_response = client.get("/health")
        assert health_response.status_code == 200
        health_data = health_response.json()
        assert health_data["status"] == "healthy"

        # Test API health endpoint
        api_health_response = client.get("/api/v1/health")
        assert api_health_response.status_code == 200
        api_health_data = api_health_response.json()
        assert api_health_data["status"] == "healthy"


if __name__ == "__main__":
    pytest.main([__file__])