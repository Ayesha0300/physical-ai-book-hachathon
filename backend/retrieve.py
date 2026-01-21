#!/usr/bin/env python3
"""
RAG Pipeline Validation Script

This script validates the RAG retrieval pipeline end-to-end by accepting a query,
retrieving relevant vectors from Qdrant, returning ranked text chunks with metadata,
and validating that retrieved content matches source URLs and expected semantics.
"""

import time
import logging
from typing import List, Optional
from config import Config
from utils.qdrant_client_module import QdrantClientUtil
from models import Query, RetrievedChunk, Metadata, ValidationResult, ValidationMetrics
from cli import parse_arguments
from logging_util import setup_logging, log_validation_result
from error_handling import (
    handle_error, validate_query_text, validate_top_k,
    validate_retrieval_result, ValidationError, RetrievalError,
    detect_empty_irrelevant_responses, handle_edge_cases
)
import cohere
from qdrant_client.http import models


def get_embeddings(text: str, cohere_client) -> List[float]:
    """
    Get embeddings for the given text using Cohere.

    Args:
        text: Text to generate embeddings for
        cohere_client: Cohere client instance

    Returns:
        List of embedding values
    """
    try:
        response = cohere_client.embed(
            texts=[text],
            model="embed-english-v3.0",
            input_type="search_query"
        )
        return response.embeddings[0]
    except Exception as e:
        raise RetrievalError(f"Failed to generate embeddings: {str(e)}")


def semantic_search(query_text: str, top_k: int, collection_name: str,
                   qdrant_util: QdrantClientUtil, cohere_client) -> List[RetrievedChunk]:
    """
    Perform semantic search using the query against the vector database.

    Args:
        query_text: The query text to search for
        top_k: Number of results to return
        collection_name: Name of the Qdrant collection to search
        qdrant_util: Qdrant client utility instance
        cohere_client: Cohere client instance

    Returns:
        List of RetrievedChunk objects
    """
    # Generate embeddings for the query
    query_vector = get_embeddings(query_text, cohere_client)

    # Perform search in Qdrant
    results = qdrant_util.search(
        vector=query_vector,
        top_k=top_k,
        filters=None  # No filters for basic validation
    )

    # Convert results to RetrievedChunk objects
    retrieved_chunks = []
    for rank, result in enumerate(results, start=1):
        # Extract payload
        payload = result.payload

        # Create metadata object
        metadata = Metadata(
            url=payload.get('url', ''),
            section=payload.get('section', ''),
            chunk_id=payload.get('chunk_id', ''),
            source_file=payload.get('source_file')
        )

        # Create RetrievedChunk object
        chunk = RetrievedChunk(
            content=payload.get('content', ''),
            similarity_score=result.score,
            metadata=metadata,
            rank=rank
        )
        retrieved_chunks.append(chunk)

    return retrieved_chunks


def validate_retrieval_pipeline(query_text: str, top_k: int = 5,
                               collection_name: Optional[str] = None) -> ValidationResult:
    """
    Validate the retrieval pipeline with the given query.

    Args:
        query_text: The query text to validate
        top_k: Number of results to retrieve
        collection_name: Name of the collection to search (optional)

    Returns:
        ValidationResult containing the results of the validation
    """
    start_time = time.time()

    # Use default collection name if not provided
    if collection_name is None:
        collection_name = Config.QDRANT_COLLECTION_NAME

    # Validate inputs
    if not validate_query_text(query_text):
        raise ValidationError("Invalid query text")

    if not validate_top_k(top_k):
        raise ValidationError("Invalid top_k value")

    try:
        # Initialize Cohere client
        cohere_client = cohere.Client(Config.COHERE_API_KEY)

        # Initialize Qdrant client utility
        qdrant_util = QdrantClientUtil(
            url=Config.QDRANT_URL,
            api_key=Config.QDRANT_API_KEY,
            collection_name=collection_name
        )

        # Perform semantic search
        retrieved_chunks = semantic_search(
            query_text, top_k, collection_name, qdrant_util, cohere_client
        )

        # Validate retrieval results
        is_valid = validate_retrieval_result(retrieved_chunks)

        # Detect empty/irrelevant responses
        if retrieved_chunks:
            issues = detect_empty_irrelevant_responses(retrieved_chunks)
            if issues:
                # Log issues but don't fail the validation, just add them to the result
                logger = logging.getLogger(__name__)
                for issue in issues:
                    logger.warning(f"Quality issue detected: {issue}")

        # Calculate execution time
        execution_time = time.time() - start_time

        # Create query object
        query = Query(text=query_text)

        # Create validation result
        result = ValidationResult(
            query=query,
            retrieved_chunks=retrieved_chunks,
            execution_time=execution_time,
            success=is_valid
        )

        # Handle edge cases
        result = handle_edge_cases(result)

        return result

    except Exception as e:
        execution_time = time.time() - start_time
        return handle_error(e, Query(text=query_text))


def execute_multiple_queries(queries: List[str], top_k: int = 5,
                           collection_name: Optional[str] = None) -> List[ValidationResult]:
    """
    Execute multiple queries and return validation results for each.

    Args:
        queries: List of query strings to execute
        top_k: Number of results to retrieve for each query
        collection_name: Name of the collection to search (optional)

    Returns:
        List of ValidationResult objects for each query
    """
    results = []
    for query_text in queries:
        try:
            result = validate_retrieval_pipeline(
                query_text=query_text,
                top_k=top_k,
                collection_name=collection_name
            )
            results.append(result)
        except Exception as e:
            # Create an error result for this specific query
            error_result = ValidationResult(
                query=Query(text=query_text),
                retrieved_chunks=[],
                execution_time=0.0,
                success=False,
                error_message=str(e)
            )
            results.append(error_result)

    return results


def calculate_validation_metrics(results: List[ValidationResult]) -> ValidationMetrics:
    """
    Calculate validation metrics from a list of validation results.

    Args:
        results: List of validation results

    Returns:
        ValidationMetrics object with calculated metrics
    """
    if not results:
        return ValidationMetrics(
            success_rate=0.0,
            avg_latency=0.0,
            avg_similarity=0.0
        )

    # Calculate success rate
    successful_results = [r for r in results if r.success]
    success_rate = len(successful_results) / len(results)

    # Calculate average latency
    total_latency = sum(r.execution_time for r in results)
    avg_latency = total_latency / len(results)

    # Calculate average similarity
    all_similarities = []
    for result in results:
        for chunk in result.retrieved_chunks:
            all_similarities.append(chunk.similarity_score)

    avg_similarity = sum(all_similarities) / len(all_similarities) if all_similarities else 0.0

    return ValidationMetrics(
        success_rate=success_rate,
        avg_latency=avg_latency,
        avg_similarity=avg_similarity
    )


def format_output(result: ValidationResult, output_format: str = "json") -> str:
    """
    Format the validation result for output.

    Args:
        result: The validation result to format
        output_format: Output format ("json", "text", or "detailed")

    Returns:
        Formatted string representation of the result
    """
    if output_format == "json":
        import json
        return json.dumps(result.to_dict(), indent=2)
    elif output_format == "detailed":
        return format_detailed_result(result)
    else:  # text format
        output = f"Query: {result.query.text}\n"
        output += f"Success: {result.success}\n"
        output += f"Execution Time: {result.execution_time:.3f}s\n"

        if result.error_message:
            output += f"Error: {result.error_message}\n"
        else:
            output += f"Retrieved {len(result.retrieved_chunks)} chunks:\n"
            for chunk in result.retrieved_chunks:
                output += f"  Rank {chunk.rank}: Score {chunk.similarity_score:.3f}\n"
                output += f"    Content: {chunk.content[:100]}...\n"
                output += f"    URL: {chunk.metadata.url}\n"
                output += f"    Section: {chunk.metadata.section}\n"

        return output


def format_detailed_result(result: ValidationResult) -> str:
    """
    Format a detailed validation result with comprehensive information.

    Args:
        result: The validation result to format

    Returns:
        Detailed string representation of the result
    """
    output = f"=== Validation Result ===\n"
    output += f"Query: {result.query.text}\n"
    output += f"Timestamp: {result.query.timestamp.isoformat()}\n"
    output += f"Success: {result.success}\n"
    output += f"Execution Time: {result.execution_time:.3f}s\n"

    if result.error_message:
        output += f"Error: {result.error_message}\n"
    else:
        output += f"Retrieved Chunks: {len(result.retrieved_chunks)}\n"
        output += f"Average Similarity: {sum(c.similarity_score for c in result.retrieved_chunks) / len(result.retrieved_chunks) if result.retrieved_chunks else 0:.3f}\n"
        output += "\n--- Detailed Chunks ---\n"
        for chunk in result.retrieved_chunks:
            output += f"Rank {chunk.rank}:\n"
            output += f"  Similarity Score: {chunk.similarity_score:.3f}\n"
            output += f"  Content: {chunk.content}\n"
            output += f"  Metadata:\n"
            output += f"    URL: {chunk.metadata.url}\n"
            output += f"    Section: {chunk.metadata.section}\n"
            output += f"    Chunk ID: {chunk.metadata.chunk_id}\n"
            if chunk.metadata.source_file:
                output += f"    Source File: {chunk.metadata.source_file}\n"
            output += "\n"

    return output


def format_batch_output(results: List[ValidationResult], output_format: str = "json") -> str:
    """
    Format multiple validation results for output.

    Args:
        results: List of validation results
        output_format: Output format ("json" or "text")

    Returns:
        Formatted string representation of the batch results
    """
    if output_format == "json":
        import json
        output_data = {
            "results": [result.to_dict() for result in results],
            "summary": calculate_validation_metrics(results).to_dict()
        }
        return json.dumps(output_data, indent=2)
    else:  # text format
        output = f"Processed {len(results)} queries:\n"
        for i, result in enumerate(results):
            output += f"\nQuery {i+1}: {result.query.text}\n"
            output += f"  Success: {result.success}\n"
            output += f"  Execution Time: {result.execution_time:.3f}s\n"

            if result.error_message:
                output += f"  Error: {result.error_message}\n"
            else:
                output += f"  Retrieved {len(result.retrieved_chunks)} chunks\n"

        # Add summary metrics
        metrics = calculate_validation_metrics(results)
        output += f"\nSummary Metrics:\n"
        output += f"  Success Rate: {metrics.success_rate:.2%}\n"
        output += f"  Avg Latency: {metrics.avg_latency:.3f}s\n"
        output += f"  Avg Similarity: {metrics.avg_similarity:.3f}\n"

        return output


def main():
    """Main entry point for the RAG pipeline validation script."""
    # Parse command line arguments
    args = parse_arguments()

    # Set up logging
    logger = setup_logging(verbose=args.verbose)

    try:
        # Validate configuration
        config_errors = Config.validate()
        if config_errors:
            for error in config_errors:
                logger.error(error)
            raise ValidationError("Configuration validation failed")

        # Check if we're running batch queries
        if args.query.startswith('@'):  # If query starts with @, treat as file containing multiple queries
            queries_file = args.query[1:]  # Remove the @ prefix
            with open(queries_file, 'r') as f:
                queries = [line.strip() for line in f if line.strip()]
        else:
            queries = [args.query]

        # Log the queries
        logger.info(f"Validating {len(queries)} query(ies)")

        if len(queries) == 1:
            # Single query mode
            logger.info(f"Validating query: {queries[0]}")

            # Validate the retrieval pipeline
            result = validate_retrieval_pipeline(
                query_text=queries[0],
                top_k=args.top_k,
                collection_name=args.collection_name
            )

            # Log the result
            log_validation_result(logger, result.to_dict(), verbose=args.verbose)

            # Format and print the output
            formatted_output = format_output(result, args.output_format)
            print(formatted_output)

            # Exit with appropriate code
            exit_code = 0 if result.success else 1
        else:
            # Batch query mode
            results = execute_multiple_queries(
                queries=queries,
                top_k=args.top_k,
                collection_name=args.collection_name
            )

            # Calculate metrics
            metrics = calculate_validation_metrics(results)

            # Log the results
            for result in results:
                log_validation_result(logger, result.to_dict(), verbose=args.verbose)

            # Log the metrics
            from logging_util import log_validation_metrics
            log_validation_metrics(logger, metrics.to_dict())

            # Format and print the batch output
            formatted_output = format_batch_output(results, args.output_format)
            print(formatted_output)

            # Exit with appropriate code based on overall success
            successful_results = [r for r in results if r.success]
            exit_code = 0 if len(successful_results) == len(queries) else 1

        return exit_code

    except Exception as e:
        logger.error(f"Application error: {str(e)}", exc_info=True)
        print(f"Error: {str(e)}")
        return 1


if __name__ == "__main__":
    exit_code = main()
    exit(exit_code)