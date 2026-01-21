"""
MCP Context7 Provider for Qdrant Vector Search
Implements the Model Context Protocol to provide vector search capabilities to the OpenAI agent.
"""
import json
import logging
from typing import Dict, Any, List, Optional
from pydantic import BaseModel
import requests
import os

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


# Load configuration directly from environment
def load_config():
    """Load configuration from environment variables."""
    class Config:
        QDRANT_URL = os.getenv('QDRANT_URL')
        QDRANT_API_KEY = os.getenv('QDRANT_API_KEY')
        QDRANT_COLLECTION_NAME = os.getenv('QDRANT_COLLECTION_NAME', 'book_docs')
        COHERE_API_KEY = os.getenv('COHERE_API_KEY')
    return Config


class VectorSearchParams(BaseModel):
    """Parameters for vector search."""
    query: str
    collection_name: str
    limit: int = 5
    filters: Optional[Dict[str, Any]] = None


class MCPContext7Provider:
    """
    MCP (Model Context Protocol) Context7 Provider for Qdrant vector search.
    Provides retrieval capabilities to the OpenAI agent via the MCP protocol.
    """

    def __init__(self):
        """Initialize the MCP Context7 provider with Qdrant configuration."""
        config = load_config()
        self.qdrant_url = config.QDRANT_URL
        self.qdrant_api_key = config.QDRANT_API_KEY
        self.collection_name = config.QDRANT_COLLECTION_NAME

        # Initialize Cohere client for embeddings
        import cohere
        self.cohere_client = cohere.Client(config.COHERE_API_KEY)

    def search_vectors(self, query: str, limit: int = 5) -> List[Dict[str, Any]]:
        """
        Search for similar vectors in Qdrant based on the query.

        Args:
            query: The text query to search for
            limit: Maximum number of results to return

        Returns:
            List of dictionaries containing the search results with content and metadata
        """
        try:
            # Embed the query using Cohere with required input_type
            query_embedding = self.cohere_client.embed(
                texts=[query],
                model="embed-multilingual-v3.0",
                input_type="search_query"
            ).embeddings[0]

            # Prepare the search request
            search_payload = {
                "vector": query_embedding,
                "limit": limit,
                "with_payload": True,
                "with_vectors": False
            }

            # Make the search request to Qdrant
            headers = {
                "Content-Type": "application/json",
                "Api-Key": self.qdrant_api_key
            }

            search_url = f"{self.qdrant_url}/collections/{self.collection_name}/points/search"
            response = requests.post(search_url, headers=headers, json=search_payload)

            if response.status_code != 200:
                logger.error(f"Qdrant search failed: {response.status_code} - {response.text}")
                return []

            search_results = response.json()

            # Format the results
            results = []
            for result in search_results.get('result', []):
                results.append({
                    'id': result.get('id'),
                    'content': result.get('payload', {}).get('content', ''),
                    'metadata': result.get('payload', {}),
                    'score': result.get('score', 0.0)
                })

            return results

        except Exception as e:
            logger.error(f"Error searching vectors in Qdrant: {e}")
            return []

    def get_context(self, query: str) -> Dict[str, Any]:
        """
        Get context for the given query following MCP Context7 protocol.

        Args:
            query: The query to get context for

        Returns:
            Dictionary containing the context information
        """
        try:
            # Perform vector search
            search_results = self.search_vectors(query, limit=5)

            # Format as MCP Context7 response
            context = {
                "content": "\n".join([result['content'] for result in search_results if result['content']]),
                "metadata": {
                    "source": "qdrant_vector_search",
                    "retrieved_chunks": len(search_results),
                    "chunks": search_results
                },
                "confidence": "high" if len(search_results) > 0 else "low"
            }

            return context

        except Exception as e:
            logger.error(f"Error getting context: {e}")
            return {
                "content": "",
                "metadata": {
                    "source": "qdrant_vector_search",
                    "error": str(e)
                },
                "confidence": "none"
            }


# Global instance of the provider
context7_provider = MCPContext7Provider()


def get_retrieval_context(query: str) -> Dict[str, Any]:
    """
    Function to get retrieval context via MCP Context7 protocol.

    Args:
        query: The query to search for

    Returns:
        Context information retrieved from Qdrant
    """
    return context7_provider.get_context(query)