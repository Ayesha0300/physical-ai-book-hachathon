from qdrant_client import QdrantClient
from qdrant_client.http import models
from typing import Optional
import logging

logger = logging.getLogger(__name__)

class QdrantClientUtil:
    """Utility class for Qdrant client connection and operations."""

    def __init__(self, url: str, api_key: str, collection_name: str):
        """
        Initialize Qdrant client utility.

        Args:
            url: Qdrant server URL
            api_key: Qdrant API key
            collection_name: Name of the collection to use
        """
        self.client = QdrantClient(
            url=url,
            api_key=api_key,
            prefer_grpc=False  # Use HTTP for better compatibility (matches main.py)
        )
        self.collection_name = collection_name

    def get_client(self) -> QdrantClient:
        """Get the Qdrant client instance."""
        return self.client

    def collection_exists(self) -> bool:
        """Check if the collection exists."""
        try:
            self.client.get_collection(self.collection_name)
            return True
        except Exception:
            return False

    def search(self, vector: list, top_k: int = 5, filters: Optional[dict] = None):
        """
        Search for similar vectors in the collection.

        Args:
            vector: Query vector to search for
            top_k: Number of results to return
            filters: Optional filters to apply to the search

        Returns:
            Search results from Qdrant
        """
        try:
            # Prepare filters if provided
            search_filter = None
            if filters:
                conditions = []
                for key, value in filters.items():
                    conditions.append(
                        models.FieldCondition(
                            key=key,
                            match=models.MatchValue(value=value)
                        )
                    )
                if conditions:
                    search_filter = models.Filter(must=conditions)

            # Perform the search
            results = self.client.search(
                collection_name=self.collection_name,
                query_vector=vector,
                limit=top_k,
                query_filter=search_filter,
                with_payload=True,
                with_vectors=False
            )

            return results
        except Exception as e:
            logger.error(f"Error searching in Qdrant: {str(e)}")
            raise