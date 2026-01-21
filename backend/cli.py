import argparse
import sys

def create_parser():
    """Create and configure the argument parser."""
    parser = argparse.ArgumentParser(
        description="RAG Pipeline Validation Tool - Validate retrieval pipeline with semantic search"
    )

    parser.add_argument(
        "--query",
        type=str,
        required=True,
        help="The natural language query to validate against the retrieval pipeline"
    )

    parser.add_argument(
        "--top-k",
        type=int,
        default=5,
        help="Number of top results to retrieve (default: 5, max: 20)"
    )

    parser.add_argument(
        "--collection-name",
        type=str,
        default=None,
        help="Name of the Qdrant collection to search (default: use config value)"
    )

    parser.add_argument(
        "--output-format",
        type=str,
        choices=["json", "text", "detailed"],
        default="json",
        help="Output format for results (default: json, options: json, text, detailed)"
    )

    parser.add_argument(
        "--verbose",
        action="store_true",
        help="Enable verbose logging output"
    )

    return parser

def parse_arguments(args=None):
    """Parse command line arguments."""
    if args is None:
        args = sys.argv[1:]

    parser = create_parser()
    parsed_args = parser.parse_args(args)

    # Validate arguments
    if parsed_args.top_k <= 0 or parsed_args.top_k > 20:
        parser.error("--top-k must be between 1 and 20")

    if not parsed_args.query.strip():
        parser.error("--query cannot be empty")

    return parsed_args