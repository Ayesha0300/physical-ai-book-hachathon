import logging
import sys
from typing import Optional

def setup_logging(verbose: bool = False) -> logging.Logger:
    """
    Set up logging configuration for the application.

    Args:
        verbose: Enable verbose logging if True

    Returns:
        Configured logger instance
    """
    logger = logging.getLogger('rag_validation')
    logger.setLevel(logging.DEBUG if verbose else logging.INFO)

    # Prevent adding multiple handlers if logger already configured
    if logger.handlers:
        return logger

    # Create console handler
    console_handler = logging.StreamHandler(sys.stdout)
    console_handler.setLevel(logging.DEBUG if verbose else logging.INFO)

    # Create formatter
    if verbose:
        formatter = logging.Formatter(
            '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
        )
    else:
        formatter = logging.Formatter(
            '%(levelname)s - %(message)s'
        )

    console_handler.setFormatter(formatter)
    logger.addHandler(console_handler)

    return logger

def log_validation_result(logger: logging.Logger, result: dict, verbose: bool = False):
    """
    Log validation results in a structured way.

    Args:
        logger: Logger instance to use
        result: Validation result dictionary
        verbose: Enable verbose logging if True
    """
    if result.get('success', False):
        logger.info(f"Query successful: {result.get('query', {}).get('text', 'N/A')}")
        logger.info(f"Execution time: {result.get('execution_time', 0):.3f}s")
        logger.info(f"Retrieved {len(result.get('retrieved_chunks', []))} chunks")

        if verbose and result.get('retrieved_chunks'):
            for i, chunk in enumerate(result['retrieved_chunks'][:3]):  # Log first 3 chunks
                logger.debug(f"Chunk {i+1}: Similarity={chunk.get('similarity_score', 0):.3f}, "
                           f"Content: {chunk.get('content', '')[:100]}...")
    else:
        logger.error(f"Query failed: {result.get('error_message', 'Unknown error')}")
        logger.error(f"Query: {result.get('query', {}).get('text', 'N/A')}")

def log_validation_metrics(logger: logging.Logger, metrics: dict):
    """
    Log validation metrics.

    Args:
        logger: Logger instance to use
        metrics: Validation metrics dictionary
    """
    logger.info("Validation Metrics:")
    logger.info(f"  Success Rate: {metrics.get('success_rate', 0):.2%}")
    logger.info(f"  Avg Latency: {metrics.get('avg_latency', 0):.3f}s")
    logger.info(f"  Avg Similarity: {metrics.get('avg_similarity', 0):.3f}")
    if metrics.get('relevance_score') is not None:
        logger.info(f"  Relevance Score: {metrics.get('relevance_score', 0):.3f}")