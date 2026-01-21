"""
Contract tests for validation output format in retrieve.py
"""
import json
from models import Query, RetrievedChunk, Metadata, ValidationResult


def test_validation_result_to_dict_format():
    """Test that ValidationResult.to_dict() produces the expected output format."""
    # Create a sample validation result
    query = Query(text="test query")
    metadata = Metadata(
        url="https://example.com",
        section="section1",
        chunk_id="chunk1",
        source_file="source.txt"
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
        success=True,
        error_message=None
    )

    # Convert to dict
    result_dict = result.to_dict()

    # Verify the structure matches the contract
    assert 'query' in result_dict
    assert 'retrieved_chunks' in result_dict
    assert 'execution_time' in result_dict
    assert 'success' in result_dict

    # Verify query structure
    assert 'text' in result_dict['query']
    assert 'timestamp' in result_dict['query']

    # Verify retrieved_chunks structure
    assert len(result_dict['retrieved_chunks']) == 1
    retrieved_chunk = result_dict['retrieved_chunks'][0]
    assert 'content' in retrieved_chunk
    assert 'similarity_score' in retrieved_chunk
    assert 'rank' in retrieved_chunk
    assert 'metadata' in retrieved_chunk

    # Verify metadata structure
    assert 'url' in retrieved_chunk['metadata']
    assert 'section' in retrieved_chunk['metadata']
    assert 'chunk_id' in retrieved_chunk['metadata']

    # Verify data types
    assert isinstance(result_dict['execution_time'], float)
    assert isinstance(result_dict['success'], bool)
    assert isinstance(retrieved_chunk['similarity_score'], float)
    assert isinstance(retrieved_chunk['rank'], int)


def test_error_result_format():
    """Test that error ValidationResult follows the expected format."""
    result = ValidationResult(
        query=Query(text="test query"),
        retrieved_chunks=[],
        execution_time=0.010,
        success=False,
        error_message="test error message"
    )

    result_dict = result.to_dict()

    # Verify error structure
    assert result_dict['success'] == False
    assert 'error_message' in result_dict
    assert result_dict['error_message'] == "test error message"
    assert result_dict['execution_time'] == 0.010


def test_json_serialization():
    """Test that validation results can be serialized to JSON."""
    result = ValidationResult(
        query=Query(text="test query"),
        retrieved_chunks=[],
        execution_time=0.123,
        success=True
    )

    # This should not raise an exception
    json_str = json.dumps(result.to_dict(), indent=2)
    assert json_str is not None

    # Parse back to verify it's valid JSON
    parsed = json.loads(json_str)
    assert 'query' in parsed
    assert 'success' in parsed