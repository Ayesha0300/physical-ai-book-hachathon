"""
Test suite for the RAG Chatbot API endpoints
"""

import pytest
import asyncio
from fastapi.testclient import TestClient
from unittest.mock import patch, MagicMock

from main import app
from api import router


@pytest.fixture
def client():
    """Create a test client for the FastAPI app"""
    with TestClient(app) as test_client:
        yield test_client


@pytest.mark.asyncio
async def test_chat_endpoint_success():
    """Test successful chat endpoint request"""
    with TestClient(app) as client:
        # Mock the RAG agent response
        with patch('api.rag_agent') as mock_agent:
            mock_agent.query.return_value = "This is a test response"

            response = client.post(
                "/api/v1/chat",
                json={
                    "query": "Test question?",
                    "selected_text": "Some selected text",
                    "restrict_to_selection": False,
                    "conversation_id": "test_conversation_123"
                }
            )

            assert response.status_code == 200
            data = response.json()
            assert data["success"] is True
            assert "data" in data
            assert "request_id" in data


@pytest.mark.asyncio
async def test_chat_endpoint_missing_query():
    """Test chat endpoint with missing query"""
    with TestClient(app) as client:
        response = client.post(
            "/api/v1/chat",
            json={
                "query": "",
                "selected_text": "Some selected text",
                "restrict_to_selection": False
            }
        )

        assert response.status_code == 400
        data = response.json()
        assert data["success"] is False
        assert data["error"]["code"] == "INVALID_QUERY"


@pytest.mark.asyncio
async def test_chat_endpoint_empty_query():
    """Test chat endpoint with empty query"""
    with TestClient(app) as client:
        response = client.post(
            "/api/v1/chat",
            json={
                "query": "   ",  # Whitespace only
                "selected_text": "Some selected text",
                "restrict_to_selection": False
            }
        )

        assert response.status_code == 400
        data = response.json()
        assert data["success"] is False
        assert data["error"]["code"] == "INVALID_QUERY"


def test_health_endpoint():
    """Test the health check endpoint"""
    with TestClient(app) as client:
        response = client.get("/health")

        assert response.status_code == 200
        data = response.json()
        assert "status" in data
        assert data["status"] == "healthy"


def test_api_health_endpoint():
    """Test the API health check endpoint"""
    with TestClient(app) as client:
        response = client.get("/api/v1/health")

        assert response.status_code == 200
        data = response.json()
        assert "status" in data
        assert data["status"] == "healthy"


@pytest.mark.asyncio
async def test_chat_endpoint_with_selected_text():
    """Test chat endpoint with selected text functionality"""
    with TestClient(app) as client:
        # Mock the RAG agent response
        with patch('api.rag_agent') as mock_agent:
            mock_agent.query.return_value = "This is a response based on selected text"

            response = client.post(
                "/api/v1/chat",
                json={
                    "query": "How does this relate to the selected text?",
                    "selected_text": "The selected text contains important information about the topic.",
                    "restrict_to_selection": True,
                    "conversation_id": "test_conversation_456"
                }
            )

            assert response.status_code == 200
            data = response.json()
            assert data["success"] is True
            assert "data" in data


@pytest.mark.asyncio
async def test_chat_endpoint_error_handling():
    """Test chat endpoint error handling"""
    with TestClient(app) as client:
        # Mock the RAG agent to raise an exception
        with patch('api.rag_agent') as mock_agent:
            mock_agent.query.side_effect = Exception("Test error")

            response = client.post(
                "/api/v1/chat",
                json={
                    "query": "Test question?",
                    "selected_text": None,
                    "restrict_to_selection": False
                }
            )

            assert response.status_code == 500
            data = response.json()
            assert data["success"] is False
            assert data["error"]["code"] == "INTERNAL_ERROR"


if __name__ == "__main__":
    pytest.main([__file__])