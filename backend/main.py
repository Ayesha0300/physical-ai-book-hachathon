"""
Main FastAPI application for the RAG Chatbot backend
"""

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
import uvicorn
import logging
import os

# Import the API router from the api module
from api import router
from config import Config

# Configure logging
logging.basicConfig(level=getattr(logging, Config.LOG_LEVEL.upper()) if hasattr(Config, 'LOG_LEVEL') else logging.INFO)

# Validate configuration on startup
try:
    # Only validate if the Config class has a validate method
    if hasattr(Config, 'validate'):
        Config.validate()
except ValueError as e:
    logging.error(f"Configuration validation failed: {e}")
    raise

# Create FastAPI app instance
app = FastAPI(
    title="RAG Chatbot API",
    description="API for the RAG (Retrieval-Augmented Generation) Chatbot system",
    version="1.0.0",
)

# Add CORS middleware for frontend-backend communication
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, specify allowed origins
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Include API routes
app.include_router(router, prefix="/api/v1")

@app.get("/")
async def root():
    """Root endpoint for health check"""
    return {"message": "RAG Chatbot Backend is running"}

@app.get("/health")
async def health_check():
    """Health check endpoint"""
    return {
        "status": "healthy",
        "api_version": "1.0.0",
        "configured_qdrant": bool(getattr(Config, 'QDRANT_URL', None))
    }

# For Vercel deployment, we need to expose the app instance
# The wsgi.py file imports this app instance
if __name__ == "__main__":
    # Run the application with uvicorn locally
    uvicorn.run(
        "main:app",
        host="0.0.0.0",
        port=int(os.getenv("PORT", 8000)),
        reload=True,
        log_level=getattr(Config, 'LOG_LEVEL', 'info').lower()
    )