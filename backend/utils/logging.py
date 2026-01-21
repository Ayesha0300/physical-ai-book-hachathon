"""
Logging utilities for the RAG Chatbot backend
Provides structured logging functionality
"""

import logging
import sys
from datetime import datetime
from typing import Dict, Any, Optional
from enum import Enum

from config import Config


class LogLevel(Enum):
    """
    Enum for log levels
    """
    DEBUG = "DEBUG"
    INFO = "INFO"
    WARNING = "WARNING"
    ERROR = "ERROR"
    CRITICAL = "CRITICAL"


class StructuredLogger:
    """
    Structured logger for the RAG Chatbot backend
    Provides consistent, structured logging across the application
    """

    def __init__(self, name: str = "rag_chatbot"):
        """
        Initialize the structured logger

        Args:
            name: Name of the logger
        """
        self.logger = logging.getLogger(name)

        # Set log level from configuration
        log_level = getattr(logging, Config.LOG_LEVEL.upper(), logging.INFO)
        self.logger.setLevel(log_level)

        # Prevent duplicate handlers
        if not self.logger.handlers:
            # Create console handler
            console_handler = logging.StreamHandler(sys.stdout)
            console_handler.setLevel(log_level)

            # Create structured formatter
            formatter = StructuredFormatter()
            console_handler.setFormatter(formatter)

            # Add handler to logger
            self.logger.addHandler(console_handler)

    def _log(self, level: LogLevel, message: str, extra_fields: Optional[Dict] = None):
        """
        Internal method to log a message with structured format

        Args:
            level: Log level
            message: Log message
            extra_fields: Additional fields to include in the log
        """
        extra_fields = extra_fields or {}
        extra_fields['timestamp'] = datetime.utcnow().isoformat()
        extra_fields['level'] = level.value

        log_method = getattr(self.logger, level.value.lower())
        log_method(message, extra={'extra_fields': extra_fields})

    def debug(self, message: str, extra_fields: Optional[Dict] = None):
        """
        Log a debug message

        Args:
            message: Log message
            extra_fields: Additional fields to include in the log
        """
        self._log(LogLevel.DEBUG, message, extra_fields)

    def info(self, message: str, extra_fields: Optional[Dict] = None):
        """
        Log an info message

        Args:
            message: Log message
            extra_fields: Additional fields to include in the log
        """
        self._log(LogLevel.INFO, message, extra_fields)

    def warning(self, message: str, extra_fields: Optional[Dict] = None):
        """
        Log a warning message

        Args:
            message: Log message
            extra_fields: Additional fields to include in the log
        """
        self._log(LogLevel.WARNING, message, extra_fields)

    def error(self, message: str, extra_fields: Optional[Dict] = None):
        """
        Log an error message

        Args:
            message: Log message
            extra_fields: Additional fields to include in the log
        """
        self._log(LogLevel.ERROR, message, extra_fields)

    def critical(self, message: str, extra_fields: Optional[Dict] = None):
        """
        Log a critical message

        Args:
            message: Log message
            extra_fields: Additional fields to include in the log
        """
        self._log(LogLevel.CRITICAL, message, extra_fields)

    def log_api_call(
        self,
        endpoint: str,
        method: str,
        status_code: int,
        duration_ms: float,
        user_id: Optional[str] = None,
        request_size: Optional[int] = None
    ):
        """
        Log an API call with structured format

        Args:
            endpoint: API endpoint that was called
            method: HTTP method used
            status_code: HTTP status code returned
            duration_ms: Duration of the API call in milliseconds
            user_id: Optional user ID
            request_size: Optional request size in bytes
        """
        extra_fields = {
            'event_type': 'api_call',
            'endpoint': endpoint,
            'method': method,
            'status_code': status_code,
            'duration_ms': duration_ms,
            'user_id': user_id,
            'request_size_bytes': request_size
        }

        message = f"API call to {method} {endpoint} completed with status {status_code}"
        self.info(message, extra_fields)

    def log_retrieval(
        self,
        query: str,
        num_results: int,
        duration_ms: float,
        context_used: Optional[str] = None
    ):
        """
        Log a retrieval operation

        Args:
            query: The query that was processed
            num_results: Number of results retrieved
            duration_ms: Duration of the retrieval in milliseconds
            context_used: Optional context used for retrieval
        """
        extra_fields = {
            'event_type': 'retrieval',
            'query_length': len(query),
            'num_results': num_results,
            'duration_ms': duration_ms,
            'context_used': bool(context_used)
        }

        message = f"Retrieved {num_results} results for query (duration: {duration_ms}ms)"
        self.info(message, extra_fields)

    def log_generation(
        self,
        query: str,
        response_length: int,
        duration_ms: float,
        model_used: str
    ):
        """
        Log a response generation operation

        Args:
            query: The original query
            response_length: Length of the generated response
            duration_ms: Duration of the generation in milliseconds
            model_used: Model used for generation
        """
        extra_fields = {
            'event_type': 'generation',
            'query_length': len(query),
            'response_length': response_length,
            'duration_ms': duration_ms,
            'model_used': model_used
        }

        message = f"Generated response of {response_length} characters (duration: {duration_ms}ms)"
        self.info(message, extra_fields)

    def log_conversation_event(
        self,
        conversation_id: str,
        event_type: str,
        user_input: Optional[str] = None,
        bot_response: Optional[str] = None
    ):
        """
        Log a conversation-related event

        Args:
            conversation_id: ID of the conversation
            event_type: Type of conversation event
            user_input: Optional user input
            bot_response: Optional bot response
        """
        extra_fields = {
            'event_type': 'conversation_event',
            'conversation_id': conversation_id,
            'event_subtype': event_type,
            'has_user_input': bool(user_input),
            'has_bot_response': bool(bot_response)
        }

        message = f"Conversation {conversation_id} event: {event_type}"
        self.info(message, extra_fields)


class StructuredFormatter(logging.Formatter):
    """
    Custom formatter that creates structured log entries
    """

    def format(self, record):
        """
        Format a log record as a structured string

        Args:
            record: The log record to format

        Returns:
            Formatted log string
        """
        # Get the basic information
        timestamp = datetime.fromtimestamp(record.created).strftime('%Y-%m-%dT%H:%M:%S.%fZ')
        level = record.levelname
        message = record.getMessage()

        # Start building the structured log
        log_parts = [
            f'[{timestamp}]',
            f'{level}',
            f'- {message}'
        ]

        # Add extra fields if they exist
        if hasattr(record, 'extra_fields') and record.extra_fields:
            extra_info = []
            for key, value in record.extra_fields.items():
                if isinstance(value, str):
                    extra_info.append(f'{key}="{value}"')
                else:
                    extra_info.append(f'{key}={value}')

            if extra_info:
                log_parts.append(f'({" ".join(extra_info)})')

        return ' '.join(log_parts)


# Global logger instance
logger = StructuredLogger()


def get_logger(name: str = "rag_chatbot") -> StructuredLogger:
    """
    Get a structured logger instance

    Args:
        name: Name of the logger (defaults to 'rag_chatbot')

    Returns:
        StructuredLogger instance
    """
    return StructuredLogger(name)


def setup_logging():
    """
    Set up logging configuration based on the application configuration
    """
    # The logger is already set up in the StructuredLogger class
    # This function can be extended to add file logging, log rotation, etc.
    pass


# Convenience functions that use the global logger
def log_debug(message: str, extra_fields: Optional[Dict] = None):
    """Log a debug message using the global logger"""
    logger.debug(message, extra_fields)


def log_info(message: str, extra_fields: Optional[Dict] = None):
    """Log an info message using the global logger"""
    logger.info(message, extra_fields)


def log_warning(message: str, extra_fields: Optional[Dict] = None):
    """Log a warning message using the global logger"""
    logger.warning(message, extra_fields)


def log_error(message: str, extra_fields: Optional[Dict] = None):
    """Log an error message using the global logger"""
    logger.error(message, extra_fields)


def log_critical(message: str, extra_fields: Optional[Dict] = None):
    """Log a critical message using the global logger"""
    logger.critical(message, extra_fields)


def log_api_call_event(
    endpoint: str,
    method: str,
    status_code: int,
    duration_ms: float,
    user_id: Optional[str] = None,
    request_size: Optional[int] = None
):
    """Log an API call event using the global logger"""
    logger.log_api_call(endpoint, method, status_code, duration_ms, user_id, request_size)


def log_retrieval_event(
    query: str,
    num_results: int,
    duration_ms: float,
    context_used: Optional[str] = None
):
    """Log a retrieval event using the global logger"""
    logger.log_retrieval(query, num_results, duration_ms, context_used)


def log_generation_event(
    query: str,
    response_length: int,
    duration_ms: float,
    model_used: str
):
    """Log a generation event using the global logger"""
    logger.log_generation(query, response_length, duration_ms, model_used)


def log_conversation_event(
    conversation_id: str,
    event_type: str,
    user_input: Optional[str] = None,
    bot_response: Optional[str] = None
):
    """Log a conversation event using the global logger"""
    logger.log_conversation_event(conversation_id, event_type, user_input, bot_response)