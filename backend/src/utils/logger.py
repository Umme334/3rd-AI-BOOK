import logging
import sys
from datetime import datetime
from typing import Any, Dict
import json
from pathlib import Path


class Logger:
    """
    Custom logging framework for backend services
    """

    def __init__(self, name: str = "textbook_generation", level: int = logging.INFO):
        self.logger = logging.getLogger(name)
        self.logger.setLevel(level)

        # Prevent adding handlers multiple times
        if not self.logger.handlers:
            # Create console handler
            console_handler = logging.StreamHandler(sys.stdout)
            console_handler.setLevel(level)

            # Create file handler
            log_dir = Path("logs")
            log_dir.mkdir(exist_ok=True)
            file_handler = logging.FileHandler(log_dir / f"{name}.log")
            file_handler.setLevel(level)

            # Create formatter
            formatter = logging.Formatter(
                '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
            )
            console_handler.setFormatter(formatter)
            file_handler.setFormatter(formatter)

            # Add handlers to logger
            self.logger.addHandler(console_handler)
            self.logger.addHandler(file_handler)

    def _log_with_context(self, level: int, message: str, context: Dict[str, Any] = None):
        """
        Log a message with optional context
        """
        if context:
            log_message = f"{message} | Context: {json.dumps(context, default=str)}"
        else:
            log_message = message

        self.logger.log(level, log_message)

    def info(self, message: str, context: Dict[str, Any] = None):
        """
        Log info level message
        """
        self._log_with_context(logging.INFO, message, context)

    def debug(self, message: str, context: Dict[str, Any] = None):
        """
        Log debug level message
        """
        self._log_with_context(logging.DEBUG, message, context)

    def warning(self, message: str, context: Dict[str, Any] = None):
        """
        Log warning level message
        """
        self._log_with_context(logging.WARNING, message, context)

    def error(self, message: str, context: Dict[str, Any] = None):
        """
        Log error level message
        """
        self._log_with_context(logging.ERROR, message, context)

    def critical(self, message: str, context: Dict[str, Any] = None):
        """
        Log critical level message
        """
        self._log_with_context(logging.CRITICAL, message, context)

    def log_api_request(self, method: str, path: str, status_code: int, response_time: float, user_id: str = None):
        """
        Log API request details
        """
        context = {
            "method": method,
            "path": path,
            "status_code": status_code,
            "response_time_ms": response_time,
            "timestamp": datetime.now().isoformat()
        }

        if user_id:
            context["user_id"] = user_id

        self.info(f"API Request: {method} {path} -> {status_code}", context)

    def log_generation_event(self, textbook_id: str, event_type: str, details: Dict[str, Any] = None):
        """
        Log textbook generation events
        """
        context = {
            "textbook_id": textbook_id,
            "event_type": event_type,
            "timestamp": datetime.now().isoformat()
        }

        if details:
            context.update(details)

        self.info(f"Generation Event: {event_type} for textbook {textbook_id}", context)

    def log_error_with_traceback(self, error: Exception, context: Dict[str, Any] = None):
        """
        Log error with full traceback
        """
        import traceback
        error_context = {
            "error_type": type(error).__name__,
            "error_message": str(error),
            "traceback": traceback.format_exc(),
            "timestamp": datetime.now().isoformat()
        }

        if context:
            error_context.update(context)

        self.error(f"Error occurred: {type(error).__name__}: {str(error)}", error_context)


# Global logger instance
logger = Logger()