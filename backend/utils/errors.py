"""
Error handling utilities for the RAG Chatbot backend
Defines custom exceptions and error handling patterns
"""

import logging
from typing import Dict, Any, Optional
from fastapi import HTTPException
from fastapi.responses import JSONResponse
from starlette.requests import Request
from starlette.responses import Response


class RAGChatbotError(Exception):
    """
    Base exception class for RAG Chatbot application
    """
    def __init__(self, message: str, error_code: str = "UNKNOWN_ERROR", details: Optional[Dict] = None):
        super().__init__(message)
        self.message = message
        self.error_code = error_code
        self.details = details or {}

    def to_dict(self) -> Dict[str, Any]:
        """
        Convert the error to a dictionary representation
        """
        return {
            "error_code": self.error_code,
            "message": self.message,
            "details": self.details
        }


class ConfigurationError(RAGChatbotError):
    """
    Exception raised when there are configuration issues
    """
    def __init__(self, message: str, details: Optional[Dict] = None):
        super().__init__(message, "CONFIGURATION_ERROR", details)


class VectorDatabaseError(RAGChatbotError):
    """
    Exception raised when there are issues with the vector database
    """
    def __init__(self, message: str, details: Optional[Dict] = None):
        super().__init__(message, "VECTOR_DATABASE_ERROR", details)


class RetrievalError(RAGChatbotError):
    """
    Exception raised when there are issues with content retrieval
    """
    def __init__(self, message: str, details: Optional[Dict] = None):
        super().__init__(message, "RETRIEVAL_ERROR", details)


class GenerationError(RAGChatbotError):
    """
    Exception raised when there are issues with response generation
    """
    def __init__(self, message: str, details: Optional[Dict] = None):
        super().__init__(message, "GENERATION_ERROR", details)


class ValidationError(RAGChatbotError):
    """
    Exception raised when there are validation issues
    """
    def __init__(self, message: str, details: Optional[Dict] = None):
        super().__init__(message, "VALIDATION_ERROR", details)


class ConversationError(RAGChatbotError):
    """
    Exception raised when there are conversation management issues
    """
    def __init__(self, message: str, details: Optional[Dict] = None):
        super().__init__(message, "CONVERSATION_ERROR", details)


async def handle_validation_error(exc: ValidationError) -> JSONResponse:
    """
    Handle validation errors and return appropriate HTTP response
    """
    logging.error(f"Validation error: {exc.message}, Details: {exc.details}")

    return JSONResponse(
        status_code=400,
        content={
            "success": False,
            "error": {
                "code": exc.error_code,
                "message": exc.message,
                "details": exc.details
            }
        }
    )


async def handle_retrieval_error(exc: RetrievalError) -> JSONResponse:
    """
    Handle retrieval errors and return appropriate HTTP response
    """
    logging.error(f"Retrieval error: {exc.message}, Details: {exc.details}")

    return JSONResponse(
        status_code=502,  # Bad Gateway - upstream service error
        content={
            "success": False,
            "error": {
                "code": exc.error_code,
                "message": exc.message,
                "details": exc.details
            }
        }
    )


async def handle_generation_error(exc: GenerationError) -> JSONResponse:
    """
    Handle generation errors and return appropriate HTTP response
    """
    logging.error(f"Generation error: {exc.message}, Details: {exc.details}")

    return JSONResponse(
        status_code=502,  # Bad Gateway - upstream service error
        content={
            "success": False,
            "error": {
                "code": exc.error_code,
                "message": exc.message,
                "details": exc.details
            }
        }
    )


async def handle_vector_database_error(exc: VectorDatabaseError) -> JSONResponse:
    """
    Handle vector database errors and return appropriate HTTP response
    """
    logging.error(f"Vector database error: {exc.message}, Details: {exc.details}")

    return JSONResponse(
        status_code=503,  # Service Unavailable
        content={
            "success": False,
            "error": {
                "code": exc.error_code,
                "message": exc.message,
                "details": exc.details
            }
        }
    )


async def handle_general_error(request: Request, exc: Exception) -> JSONResponse:
    """
    Handle general/unexpected errors and return appropriate HTTP response
    """
    logging.error(f"Unexpected error: {str(exc)}", exc_info=True)

    # Don't expose internal error details to the client
    return JSONResponse(
        status_code=500,
        content={
            "success": False,
            "error": {
                "code": "INTERNAL_ERROR",
                "message": "An internal server error occurred",
                "details": {}
            }
        }
    )


def log_error(error: RAGChatbotError, context: Optional[Dict] = None):
    """
    Log an error with appropriate context

    Args:
        error: The error to log
        context: Additional context information
    """
    log_context = context or {}
    log_context.update({
        "error_code": error.error_code,
        "error_message": error.message,
        "error_details": error.details
    })

    logging.error(f"RAG Chatbot Error: {error.message}", extra={"extra_fields": log_context})


def create_error_response(
    error_code: str,
    message: str,
    status_code: int = 500,
    details: Optional[Dict] = None
) -> JSONResponse:
    """
    Create a standardized error response

    Args:
        error_code: The error code
        message: The error message
        status_code: The HTTP status code
        details: Additional error details

    Returns:
        JSONResponse with standardized error format
    """
    return JSONResponse(
        status_code=status_code,
        content={
            "success": False,
            "error": {
                "code": error_code,
                "message": message,
                "details": details or {}
            }
        }
    )


# Exception handlers for FastAPI
exception_handlers = {
    ValidationError: handle_validation_error,
    RetrievalError: handle_retrieval_error,
    GenerationError: handle_generation_error,
    VectorDatabaseError: handle_vector_database_error,
    Exception: handle_general_error,
}


def register_error_handlers(app):
    """
    Register error handlers with a FastAPI application

    Args:
        app: FastAPI application instance
    """
    for exc_class, handler_func in exception_handlers.items():
        app.add_exception_handler(exc_class, handler_func)

    logging.info("Registered error handlers with FastAPI application")