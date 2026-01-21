"""
Integration tests for batch validation in retrieve.py
"""
import pytest
from models import Query, RetrievedChunk, Metadata, ValidationResult, ValidationMetrics
from retrieve import calculate_validation_metrics


def test_calculate_validation_metrics_all_success():
    """Test validation metrics calculation when all queries succeed."""
    # Create sample results
    results = [
        ValidationResult(
            query=Query(text="query 1"),
            retrieved_chunks=[
                RetrievedChunk("content1", 0.8, Metadata("url1", "section1", "chunk1"), 1),
                RetrievedChunk("content2", 0.7, Metadata("url2", "section2", "chunk2"), 2)
            ],
            execution_time=0.1,
            success=True
        ),
        ValidationResult(
            query=Query(text="query 2"),
            retrieved_chunks=[
                RetrievedChunk("content3", 0.9, Metadata("url3", "section3", "chunk3"), 1)
            ],
            execution_time=0.2,
            success=True
        )
    ]

    metrics = calculate_validation_metrics(results)

    # Verify metrics
    assert metrics.success_rate == 1.0  # 100% success rate
    assert abs(metrics.avg_latency - 0.15) < 0.001  # (0.1 + 0.2) / 2
    assert abs(metrics.avg_similarity - 0.8) < 0.001  # (0.8 + 0.7 + 0.9) / 3


def test_calculate_validation_metrics_with_failures():
    """Test validation metrics calculation when some queries fail."""
    results = [
        ValidationResult(
            query=Query(text="query 1"),
            retrieved_chunks=[
                RetrievedChunk("content1", 0.8, Metadata("url1", "section1", "chunk1"), 1)
            ],
            execution_time=0.1,
            success=True
        ),
        ValidationResult(
            query=Query(text="query 2"),
            retrieved_chunks=[],
            execution_time=0.0,
            success=False,
            error_message="Test error"
        )
    ]

    metrics = calculate_validation_metrics(results)

    # Verify metrics
    assert metrics.success_rate == 0.5  # 50% success rate
    assert metrics.avg_latency == 0.05  # (0.1 + 0.0) / 2
    assert metrics.avg_similarity == 0.8  # Only successful queries with chunks contribute to similarity


def test_calculate_validation_metrics_empty_results():
    """Test validation metrics calculation with empty results list."""
    metrics = calculate_validation_metrics([])

    # Verify default values
    assert metrics.success_rate == 0.0
    assert metrics.avg_latency == 0.0
    assert metrics.avg_similarity == 0.0


def test_calculate_validation_metrics_no_chunks():
    """Test validation metrics calculation when queries have no chunks."""
    results = [
        ValidationResult(
            query=Query(text="query 1"),
            retrieved_chunks=[],
            execution_time=0.1,
            success=True
        ),
        ValidationResult(
            query=Query(text="query 2"),
            retrieved_chunks=[],
            execution_time=0.2,
            success=True
        )
    ]

    metrics = calculate_validation_metrics(results)

    # Verify metrics
    assert metrics.success_rate == 1.0  # 100% success rate
    assert abs(metrics.avg_latency - 0.15) < 0.001  # (0.1 + 0.2) / 2
    assert metrics.avg_similarity == 0.0  # No chunks, so no similarity


def test_validation_metrics_to_dict():
    """Test that ValidationMetrics.to_dict() produces the expected format."""
    metrics = ValidationMetrics(
        success_rate=0.95,
        avg_latency=0.123,
        avg_similarity=0.85,
        relevance_score=0.78
    )

    metrics_dict = metrics.to_dict()

    # Verify the structure
    assert 'success_rate' in metrics_dict
    assert 'avg_latency' in metrics_dict
    assert 'avg_similarity' in metrics_dict
    assert 'relevance_score' in metrics_dict

    # Verify values
    assert metrics_dict['success_rate'] == 0.95
    assert metrics_dict['avg_latency'] == 0.123
    assert metrics_dict['avg_similarity'] == 0.85
    assert metrics_dict['relevance_score'] == 0.78