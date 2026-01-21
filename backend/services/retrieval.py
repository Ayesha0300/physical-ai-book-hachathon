"""
Vector Retrieval Service for the RAG Chatbot backend
Handles retrieval of relevant content from the vector database
"""

import asyncio
import logging
from typing import List, Optional
from datetime import datetime

from qdrant_client import QdrantClient
from qdrant_client.http import models
from qdrant_client.http.models import PointStruct
from pydantic import BaseModel

from models.response import VectorSearchResult
from config import Config


class VectorRetrievalService:
    """
    Service for retrieving relevant content from the vector database (Qdrant)
    """

    def __init__(self):
        """
        Initialize the retrieval service with Qdrant client
        """
        try:
            # Initialize Qdrant client using configuration
            if "localhost" in Config.QDRANT_URL or "127.0.0.1" in Config.QDRANT_URL:
                # For local Qdrant instance
                self.client = QdrantClient(
                    url=Config.QDRANT_URL,
                    prefer_grpc=False,
                    timeout=30
                )
            else:
                # For Qdrant Cloud
                self.client = QdrantClient(
                    url=Config.QDRANT_URL,
                    api_key=Config.QDRANT_API_KEY,
                    prefer_grpc=False,
                    timeout=30
                )

            self.collection_name = Config.QDRANT_COLLECTION_NAME
            logging.info(f"Vector retrieval service initialized with collection: {self.collection_name}")
        except Exception as e:
            logging.error(f"Failed to initialize Qdrant client: {e}")
            raise

    async def retrieve_relevant_content(
        self,
        query: str,
        top_k: int = 5,
        restrict_to_selection: bool = False,
        selected_text: Optional[str] = None
    ) -> List[VectorSearchResult]:
        """
        Retrieve relevant content from the vector database based on the query

        Args:
            query: The search query
            top_k: Number of top results to return
            restrict_to_selection: Whether to limit search to selected text
            selected_text: Text to restrict the search to

        Returns:
            List of VectorSearchResult objects
        """
        try:
            # Log the retrieval request
            logging.info(f"Retrieving content for query: {query[:100]}...")

            # In a real implementation, we would:
            # 1. Generate embeddings for the query
            # 2. Search the vector database
            # 3. Return the results

            # For now, simulate the retrieval process
            # In a full implementation, you would use a service like Cohere or OpenAI to embed the query
            # and then search the Qdrant database

            # If restricting to selection, modify the search strategy
            search_text = query
            if selected_text and restrict_to_selection:
                search_text = f"{query} {selected_text}"

            # Perform the vector search in Qdrant
            # This would require the query to be embedded first
            # For simulation, we'll return mock results
            results = await self._perform_vector_search(search_text, top_k)

            logging.info(f"Retrieved {len(results)} relevant content pieces")

            return results

        except Exception as e:
            logging.error(f"Error retrieving content for query '{query[:50]}...': {str(e)}")
            # Return empty list in case of error
            return []

    async def _perform_vector_search(self, query_text: str, top_k: int) -> List[VectorSearchResult]:
        """
        Perform the actual vector search in Qdrant

        Args:
            query_text: Text to search for
            top_k: Number of top results to return

        Returns:
            List of VectorSearchResult objects
        """
        try:
            # In a real implementation, we would:
            # 1. Use an embedding model to convert query_text to a vector
            # 2. Search the Qdrant collection for similar vectors
            # 3. Return the results as VectorSearchResult objects

            # For now, we'll return mock results
            # In a real implementation, you would need to generate embeddings for the query
            # using the same model that was used to create the stored embeddings

            # Mock results - in real implementation these would come from Qdrant
            mock_results = [
                VectorSearchResult(
                    content=f"Mock content related to: {query_text}",
                    score=0.9,
                    metadata={"source": "mock_document", "page": 1},
                    document_id=f"mock_doc_{i}"
                )
                for i in range(min(top_k, 3))  # Return up to 3 mock results
            ]

            return mock_results

        except Exception as e:
            logging.error(f"Error performing vector search: {str(e)}")
            return []

    async def search_by_text_selection(
        self,
        query: str,
        selected_text: str,
        top_k: int = 3
    ) -> List[VectorSearchResult]:
        """
        Search specifically within the selected text

        Args:
            query: The search query
            selected_text: Text to search within
            top_k: Number of top results to return

        Returns:
            List of VectorSearchResult objects
        """
        try:
            # This would implement a more targeted search within the selected text
            # For now, we'll use the same approach as the general search
            # but with emphasis on the selected text

            enhanced_query = f"Focus on: {selected_text}. Question: {query}"

            results = await self._perform_vector_search(enhanced_query, top_k)

            logging.info(f"Searched within selected text, got {len(results)} results")

            return results

        except Exception as e:
            logging.error(f"Error searching within selected text: {str(e)}")
            return []

    async def validate_retrieval_quality(
        self,
        query: str,
        results: List[VectorSearchResult]
    ) -> bool:
        """
        Validate the quality of retrieved results

        Args:
            query: Original query
            results: Retrieved results to validate

        Returns:
            Boolean indicating if results are of sufficient quality
        """
        try:
            # Check if we have results and if they have sufficient scores
            if not results:
                return False

            # Check if average score is above threshold
            avg_score = sum(result.score for result in results) / len(results)
            quality_threshold = 0.5  # Adjustable threshold

            is_quality_high = avg_score >= quality_threshold

            logging.info(f"Retrieval quality check: avg_score={avg_score:.2f}, quality_ok={is_quality_high}")

            return is_quality_high

        except Exception as e:
            logging.error(f"Error validating retrieval quality: {str(e)}")
            return False


# Async wrapper for use in endpoints
async def get_relevant_content(
    query: str,
    top_k: int = 5,
    restrict_to_selection: bool = False,
    selected_text: Optional[str] = None
) -> List[VectorSearchResult]:
    """
    Convenience function to get relevant content with default service
    """
    service = VectorRetrievalService()
    return await service.retrieve_relevant_content(
        query, top_k, restrict_to_selection, selected_text
    )