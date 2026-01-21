"""
Unit tests for logging functionality in logging_util.py
"""
import logging
from logging_util import setup_logging, log_validation_result, log_validation_metrics
from models import Query, RetrievedChunk, Metadata, ValidationResult, ValidationMetrics


def test_setup_logging_basic():
    """Test basic logging setup."""
    logger = setup_logging()
    assert logger is not None
    assert logger.name == 'rag_validation'
    assert logger.level == logging.INFO  # Default level


def test_setup_logging_verbose():
    """Test verbose logging setup."""
    logger = setup_logging(verbose=True)
    assert logger is not None
    assert logger.level == logging.DEBUG  # Verbose level


def test_log_validation_result():
    """Test logging of validation results."""
    logger = setup_logging()

    # Create a sample validation result
    query = Query(text="test query")
    metadata = Metadata(
        url="https://example.com",
        section="section1",
        chunk_id="chunk1"
    )
    chunk = RetrievedChunk(
        content="test content",
        similarity_score=0.85,
        metadata=metadata,
        rank=1
    )
    result = ValidationResult(
        query=query,
        retrieved_chunks=[chunk],
        execution_time=0.123,
        success=True
    )

    # This should not raise an exception
    log_validation_result(logger, result.to_dict())
    log_validation_result(logger, result.to_dict(), verbose=True)


def test_log_validation_metrics():
    """Test logging of validation metrics."""
    logger = setup_logging()

    # Create sample metrics
    metrics = ValidationMetrics(
        success_rate=0.95,
        avg_latency=0.123,
        avg_similarity=0.85
    )

    # This should not raise an exception
    log_validation_metrics(logger, metrics.to_dict())