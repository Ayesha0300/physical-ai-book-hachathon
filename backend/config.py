import os
from dotenv import load_dotenv

# Load environment variables from .env file
load_dotenv()

class Config:
    """
    Configuration class for the RAG agent that loads settings from environment variables.
    """

    # Load environment variables with defaults
    COHERE_API_KEY = os.getenv('COHERE_API_KEY')
    QDRANT_URL = os.getenv('QDRANT_URL')
    QDRANT_API_KEY = os.getenv('QDRANT_API_KEY')
    QDRANT_COLLECTION_NAME = os.getenv('QDRANT_COLLECTION_NAME', 'book_docs')
    GEMINI_API_KEY = os.getenv('GEMINI_API_KEY', os.getenv('GOOGLE_API_KEY'))  # Fallback to GOOGLE_API_KEY
    GEMINI_MODEL = os.getenv('GEMINI_MODEL', 'gemini-2.5-flash')
    OPENAI_API_KEY = os.getenv('OPENAI_API_KEY')
    OPENAI_MODEL = os.getenv('OPENAI_MODEL', 'gpt-4-turbo')
    CHUNK_SIZE = int(os.getenv('CHUNK_SIZE', '512'))
    CHUNK_OVERLAP = int(os.getenv('CHUNK_OVERLAP', '50'))
    REQUEST_TIMEOUT = int(os.getenv('REQUEST_TIMEOUT', '30'))
    BATCH_SIZE = int(os.getenv('BATCH_SIZE', '10'))
    SITEMAP_URL = os.getenv('SITEMAP_URL', 'https://physical-ai-book-hachathon.vercel.app/sitemap.xml')

    @classmethod
    def validate(cls):
        """
        Validate that all required configuration values are present.

        Raises:
            ValueError: If any required configuration is missing
        """
        required_configs = [
            ('COHERE_API_KEY', cls.COHERE_API_KEY),
            ('QDRANT_URL', cls.QDRANT_URL),
            ('QDRANT_API_KEY', cls.QDRANT_API_KEY),
            ('OPENROUTER_API_KEY', os.getenv('OPENROUTER_API_KEY'))
        ]

        missing_configs = []
        for name, value in required_configs:
            if not value:
                missing_configs.append(name)

        if missing_configs:
            raise ValueError(f"Missing required configuration(s): {', '.join(missing_configs)}")