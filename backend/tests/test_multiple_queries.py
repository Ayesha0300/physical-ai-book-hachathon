"""
Unit tests for multiple query execution in retrieve.py
"""
import pytest
from unittest.mock import Mock, patch
from models import Query, RetrievedChunk, Metadata, ValidationResult
from retrieve import execute_multiple_queries


def test_execute_multiple_queries_success():
    """Test successful execution of multiple queries."""
    # Mock the validate_retrieval_pipeline function
    with patch('retrieve.validate_retrieval_pipeline') as mock_validate:
        # Setup mock responses for each query
        mock_result1 = ValidationResult(
            query=Query(text="query 1"),
            retrieved_chunks=[],
            execution_time=0.1,
            success=True
        )
        mock_result2 = ValidationResult(
            query=Query(text="query 2"),
            retrieved_chunks=[],
            execution_time=0.2,
            success=True
        )

        # Mock the function to return different results for different calls
        mock_validate.side_effect = [mock_result1, mock_result2]

        # Execute multiple queries
        queries = ["query 1", "query 2"]
        results = execute_multiple_queries(queries, top_k=1)

        # Verify the results
        assert len(results) == 2
        assert results[0].query.text == "query 1"
        assert results[1].query.text == "query 2"
        assert results[0].success == True
        assert results[1].success == True


def test_execute_multiple_queries_with_error():
    """Test execution of multiple queries where one fails."""
    from error_handling import ValidationError

    # Mock the validate_retrieval_pipeline function
    with patch('retrieve.validate_retrieval_pipeline') as mock_validate:
        # Setup mock responses - first succeeds, second fails
        mock_result1 = ValidationResult(
            query=Query(text="query 1"),
            retrieved_chunks=[],
            execution_time=0.1,
            success=True
        )

        # First call succeeds, second raises exception
        mock_validate.side_effect = [mock_result1, ValidationError("Test error")]

        # Execute multiple queries
        queries = ["query 1", "query 2"]
        results = execute_multiple_queries(queries, top_k=1)

        # Verify the results
        assert len(results) == 2
        assert results[0].success == True
        assert results[1].success == False
        assert results[1].error_message is not None


def test_execute_multiple_queries_empty_list():
    """Test execution of empty query list."""
    results = execute_multiple_queries([], top_k=1)

    # Should return empty list
    assert len(results) == 0


def test_execute_multiple_queries_single_query():
    """Test execution of single query in the multiple queries function."""
    with patch('retrieve.validate_retrieval_pipeline') as mock_validate:
        mock_result = ValidationResult(
            query=Query(text="single query"),
            retrieved_chunks=[],
            execution_time=0.1,
            success=True
        )
        mock_validate.return_value = mock_result

        results = execute_multiple_queries(["single query"], top_k=1)

        assert len(results) == 1
        assert results[0].query.text == "single query"