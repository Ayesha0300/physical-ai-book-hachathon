"""
Integration tests for result formatting in retrieve.py
"""
from models import Query, RetrievedChunk, Metadata, ValidationResult
from retrieve import format_output, format_detailed_result


def test_format_output_json():
    """Test JSON output formatting."""
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

    formatted = format_output(result, "json")

    # Should contain JSON structure
    assert '"query"' in formatted
    assert '"retrieved_chunks"' in formatted
    assert '"execution_time"' in formatted
    assert '"success"' in formatted


def test_format_output_text():
    """Test text output formatting."""
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

    formatted = format_output(result, "text")

    # Should contain expected text elements
    assert "Query:" in formatted
    assert "Success:" in formatted
    assert "Execution Time:" in formatted
    assert "Retrieved 1 chunks:" in formatted


def test_format_output_detailed():
    """Test detailed output formatting."""
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

    formatted = format_output(result, "detailed")

    # Should contain detailed format elements
    assert "=== Validation Result ===" in formatted
    assert "Timestamp:" in formatted
    assert "Rank 1:" in formatted
    assert "Similarity Score:" in formatted


def test_format_detailed_result():
    """Test detailed result formatting function directly."""
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

    formatted = format_detailed_result(result)

    # Should contain detailed format elements
    assert "=== Validation Result ===" in formatted
    assert "Query:" in formatted
    assert "Timestamp:" in formatted
    assert "Rank 1:" in formatted
    assert "Content:" in formatted
    assert "Metadata:" in formatted
    assert "URL:" in formatted
    assert "Section:" in formatted


def test_format_output_error_result():
    """Test output formatting for error results."""
    query = Query(text="test query")
    result = ValidationResult(
        query=query,
        retrieved_chunks=[],
        execution_time=0.010,
        success=False,
        error_message="Test error"
    )

    formatted = format_output(result, "text")

    # Should contain error information
    assert "Error:" in formatted
    assert "Test error" in formatted