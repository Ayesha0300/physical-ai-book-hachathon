import logging
from typing import Any, Dict, Optional, List
from models import ValidationResult, Query, RetrievedChunk

logger = logging.getLogger(__name__)

class ValidationError(Exception):
    """Custom exception for validation errors."""
    pass

class RetrievalError(Exception):
    """Custom exception for retrieval errors."""
    pass

def handle_error(error: Exception, query: Optional[Query] = None) -> ValidationResult:
    """
    Handle errors and create a validation result with error information.

    Args:
        error: The exception that occurred
        query: The query that was being processed (if available)

    Returns:
        ValidationResult with error information
    """
    logger.error(f"Error occurred during validation: {str(error)}", exc_info=True)

    # Sanitize error message to avoid exposing sensitive information
    error_str = str(error)
    sanitized_error = sanitize_error_message(error_str)

    return ValidationResult(
        query=query or Query(text="Unknown query"),
        retrieved_chunks=[],
        execution_time=0.0,
        success=False,
        error_message=sanitized_error
    )


def sanitize_error_message(error_msg: str) -> str:
    """
    Sanitize error messages to prevent exposing sensitive information.

    Args:
        error_msg: The original error message

    Returns:
        Sanitized error message safe for display
    """
    # Remove potential sensitive information from error messages
    sanitized = error_msg

    # Remove API keys if present
    import re
    sanitized = re.sub(r'[A-Za-z0-9]{20,}', '[REDACTED]', sanitized)

    # Remove URLs that might contain sensitive parameters
    sanitized = re.sub(r'https?://[^\s\'"<>]+', '[URL_REDACTED]', sanitized)

    # Remove potential file paths that might expose system structure
    sanitized = re.sub(r'([A-Za-z]:)?[\\/][^:\n]*\.(py|env|json|yaml|yml)', '[FILE_PATH_REDACTED]', sanitized)

    return sanitized

def validate_retrieval_result(chunks: list) -> bool:
    """
    Validate that the retrieval result meets quality criteria.

    Args:
        chunks: List of retrieved chunks

    Returns:
        True if validation passes, False otherwise
    """
    if not chunks:
        logger.warning("No chunks retrieved for query")
        return False

    # Validate each chunk
    for chunk in chunks:
        if hasattr(chunk, 'content') and not chunk.content.strip():
            logger.warning("Retrieved chunk has empty content")
            return False

        if hasattr(chunk, 'similarity_score'):
            score = chunk.similarity_score
            if not (0.0 <= score <= 1.0):
                logger.warning(f"Invalid similarity score: {score}")
                return False

    return True


def handle_edge_cases(result: ValidationResult) -> ValidationResult:
    """
    Handle edge cases in validation results.

    Args:
        result: The validation result to check for edge cases

    Returns:
        ValidationResult with edge cases handled appropriately
    """
    # Check for extremely high execution time (possible timeout or performance issue)
    if result.execution_time > 10.0:  # More than 10 seconds
        logger.warning(f"Extremely high execution time: {result.execution_time:.3f}s")

    # Check for very low similarity scores across all chunks
    if result.retrieved_chunks:
        avg_similarity = sum(c.similarity_score for c in result.retrieved_chunks) / len(result.retrieved_chunks)
        if avg_similarity < 0.1:  # Very low similarity
            logger.warning(f"Very low average similarity score: {avg_similarity:.3f}")
            # Consider this as a potential failure in retrieval quality
            if result.success and avg_similarity < 0.05:
                result.success = False
                if not result.error_message:
                    result.error_message = "Low quality retrieval results"

    # Check for empty results that might be valid
    if not result.retrieved_chunks and result.success:
        logger.info("Query returned no results but was marked as successful")

    return result


def detect_empty_irrelevant_responses(chunks: List['RetrievedChunk'],
                                   min_similarity_threshold: float = 0.3,
                                   min_content_length: int = 20) -> List[str]:
    """
    Detect empty or irrelevant responses in retrieved chunks.

    Args:
        chunks: List of retrieved chunks to analyze
        min_similarity_threshold: Minimum similarity score to consider relevant
        min_content_length: Minimum content length to consider non-empty

    Returns:
        List of issues found in the chunks
    """
    issues = []

    for i, chunk in enumerate(chunks):
        # Check for empty or very short content
        if len(chunk.content.strip()) < min_content_length:
            issues.append(f"Chunk {i+1}: Content too short ({len(chunk.content)} chars)")

        # Check for low similarity scores
        if chunk.similarity_score < min_similarity_threshold:
            issues.append(f"Chunk {i+1}: Low similarity score ({chunk.similarity_score:.3f})")

        # Check for empty metadata fields that are important
        if not chunk.metadata.url:
            issues.append(f"Chunk {i+1}: Missing URL in metadata")

        if not chunk.metadata.section:
            issues.append(f"Chunk {i+1}: Missing section in metadata")

    return issues

def validate_query_text(query_text: str) -> bool:
    """
    Validate that the query text is appropriate for processing.

    Args:
        query_text: The query text to validate

    Returns:
        True if validation passes, False otherwise
    """
    if not query_text or not query_text.strip():
        logger.error("Query text is empty or contains only whitespace")
        return False

    if len(query_text.strip()) < 3:
        logger.warning("Query text is very short (< 3 characters)")
        return False

    return True

def validate_top_k(top_k: int) -> bool:
    """
    Validate that the top_k parameter is within acceptable bounds.

    Args:
        top_k: The number of results to retrieve

    Returns:
        True if validation passes, False otherwise
    """
    if top_k <= 0:
        logger.error("top_k must be greater than 0")
        return False

    if top_k > 20:
        logger.error("top_k must not exceed 20")
        return False

    return True