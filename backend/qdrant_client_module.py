from qdrant_client import QdrantClient
from qdrant_client.http import models
from typing import List, Dict, Optional
import logging

class QdrantRAGClient:
    """Qdrant client for RAG operations including vector search and retrieval."""

    def __init__(self, url: str, api_key: str, collection_name: str):
        """
        Initialize the Qdrant client with connection parameters.

        Args:
            url: Qdrant instance URL
            api_key: Qdrant API key
            collection_name: Name of the collection to use for RAG
        """
        self.client = QdrantClient(
            url=url,
            api_key=api_key,
            prefer_grpc=False  # Use HTTP for better compatibility
        )
        self.collection_name = collection_name

    def search_vectors(self, query_vector: List[float], limit: int = 5) -> List[Dict]:
        """
        Search for similar vectors in the Qdrant collection.

        Args:
            query_vector: The vector to search for similarity
            limit: Maximum number of results to return

        Returns:
            List of dictionaries containing the search results
        """
        try:
            search_results = self.client.search(
                collection_name=self.collection_name,
                query_vector=query_vector,
                limit=limit
            )

            # Extract relevant information from search results
            results = []
            for result in search_results:
                results.append({
                    'id': result.id,
                    'payload': result.payload,
                    'score': result.score
                })

            return results

        except Exception as e:
            logging.error(f"Error searching vectors in Qdrant: {e}")
            return []

    def get_document_by_id(self, doc_id: str) -> Optional[Dict]:
        """
        Retrieve a specific document by its ID.

        Args:
            doc_id: ID of the document to retrieve

        Returns:
            Document payload if found, None otherwise
        """
        try:
            records = self.client.retrieve(
                collection_name=self.collection_name,
                ids=[doc_id]
            )

            if records:
                record = records[0]
                return {
                    'id': record.id,
                    'payload': record.payload
                }
            return None

        except Exception as e:
            logging.error(f"Error retrieving document by ID: {e}")
            return None

    def check_connection(self) -> bool:
        """
        Check if the connection to Qdrant is working.

        Returns:
            True if connection is successful, False otherwise
        """
        try:
            # Try to get collection info to verify connection
            collection_info = self.client.get_collection(self.collection_name)
            return True
        except Exception as e:
            logging.error(f"Error connecting to Qdrant: {e}")
            return False
