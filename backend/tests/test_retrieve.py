"""
Unit tests for the retrieval function in retrieve.py
"""
import pytest
from unittest.mock import Mock, patch
from models import Query, RetrievedChunk, Metadata, ValidationResult
from retrieve import validate_retrieval_pipeline


def test_validate_retrieval_pipeline_success():
    """Test successful retrieval pipeline validation."""
    # This test would require mocking external dependencies like Cohere and Qdrant
    # For now, we'll test with mocked responses

    # Mock the external dependencies
    with patch('retrieve.cohere.Client') as mock_cohere, \
         patch('retrieve.QdrantClientUtil') as mock_qdrant:

        # Setup mock responses
        mock_cohere_instance = Mock()
        mock_cohere_instance.embed.return_value = Mock(embeddings=[[0.1, 0.2, 0.3]])
        mock_cohere.return_value = mock_cohere_instance

        mock_qdrant_instance = Mock()
        mock_qdrant_instance.search.return_value = [
            Mock(payload={'content': 'test content', 'url': 'http://example.com',
                        'section': 'test', 'chunk_id': '1'}, score=0.8)
        ]
        mock_qdrant.return_value = mock_qdrant_instance

        # Mock Config values
        with patch('retrieve.Config') as mock_config:
            mock_config.QDRANT_URL = "test_url"
            mock_config.QDRANT_API_KEY = "test_key"
            mock_config.COHERE_API_KEY = "test_key"
            mock_config.QDRANT_COLLECTION_NAME = "test_collection"

            # Run the validation
            result = validate_retrieval_pipeline("test query", top_k=1)

            # Assertions
            assert isinstance(result, ValidationResult)
            assert result.success == True
            assert result.query.text == "test query"
            assert len(result.retrieved_chunks) == 1


def test_validate_retrieval_pipeline_empty_query():
    """Test retrieval pipeline with empty query."""
    from error_handling import ValidationError

    try:
        result = validate_retrieval_pipeline("", top_k=1)
        assert False, "Expected ValidationError for empty query"
    except ValidationError:
        # Expected behavior
        pass


def test_validate_retrieval_pipeline_invalid_top_k():
    """Test retrieval pipeline with invalid top_k value."""
    from error_handling import ValidationError

    try:
        result = validate_retrieval_pipeline("test query", top_k=0)
        assert False, "Expected ValidationError for invalid top_k"
    except ValidationError:
        # Expected behavior
        pass


def test_validate_retrieval_pipeline_negative_top_k():
    """Test retrieval pipeline with negative top_k value."""
    from error_handling import ValidationError

    try:
        result = validate_retrieval_pipeline("test query", top_k=-1)
        assert False, "Expected ValidationError for negative top_k"
    except ValidationError:
        # Expected behavior
        pass