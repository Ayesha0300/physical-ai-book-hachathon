from dataclasses import dataclass
from typing import List, Optional, Dict, Any
from datetime import datetime

@dataclass
class Metadata:
    """Metadata for a retrieved chunk."""
    url: str
    section: str
    chunk_id: str
    source_file: Optional[str] = None

    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary representation."""
        return {
            "url": self.url,
            "section": self.section,
            "chunk_id": self.chunk_id,
            "source_file": self.source_file
        }

@dataclass
class RetrievedChunk:
    """A chunk retrieved from the vector database."""
    content: str
    similarity_score: float
    metadata: Metadata
    rank: int

    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary representation."""
        return {
            "content": self.content,
            "similarity_score": self.similarity_score,
            "rank": self.rank,
            "metadata": self.metadata.to_dict()
        }

@dataclass
class Query:
    """A query to validate against the retrieval pipeline."""
    text: str
    timestamp: datetime = None

    def __post_init__(self):
        """Set timestamp if not provided."""
        if self.timestamp is None:
            self.timestamp = datetime.now()

    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary representation."""
        return {
            "text": self.text,
            "timestamp": self.timestamp.isoformat()
        }

@dataclass
class ValidationResult:
    """Result of a validation query."""
    query: Query
    retrieved_chunks: List[RetrievedChunk]
    execution_time: float
    success: bool
    error_message: Optional[str] = None

    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary representation."""
        return {
            "query": self.query.to_dict(),
            "retrieved_chunks": [chunk.to_dict() for chunk in self.retrieved_chunks],
            "execution_time": self.execution_time,
            "success": self.success,
            "error_message": self.error_message
        }

@dataclass
class ValidationMetrics:
    """Metrics for validation results."""
    success_rate: float
    avg_latency: float
    avg_similarity: float
    relevance_score: Optional[float] = None

    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary representation."""
        return {
            "success_rate": self.success_rate,
            "avg_latency": self.avg_latency,
            "avg_similarity": self.avg_similarity,
            "relevance_score": self.relevance_score
        }