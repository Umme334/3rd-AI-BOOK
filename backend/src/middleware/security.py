from fastapi import Request, HTTPException
from typing import Dict, Any, List
import re
import html
import urllib.parse
from ..utils.logger import logger


class SecurityMiddleware:
    """
    Security middleware for input validation and protection against common attacks
    """

    def __init__(self):
        # Define patterns for common attacks
        self.xss_patterns = [
            r'<script.*?>.*?</script>',
            r'javascript:',
            r'on\w+\s*=',
            r'<iframe.*?>',
            r'<object.*?>',
            r'<embed.*?>',
        ]

        # SQL injection patterns
        self.sql_injection_patterns = [
            r"(\b(SELECT|INSERT|UPDATE|DELETE|DROP|CREATE|ALTER|EXEC|UNION|UNION\s+ALL)\b)",
            r"(\b(OR|AND)\s+.*?=.*?\b)",
            r"(\b(OR|AND)\s+\d+\s*=\s*\d+\b)",  # Always true conditions
            r"(--)",  # SQL comment
            r"(\b(OR|AND)\s+\d+\s*[=<>]\s*\d+\b)",  # SQL injection conditions
        ]

        # Define allowed content types
        self.allowed_content_types = [
            "application/json",
            "application/x-www-form-urlencoded",
            "multipart/form-data",
            "text/plain",
        ]

    def validate_input(self, data: str) -> bool:
        """
        Validate input against common attack patterns
        """
        if not data or not isinstance(data, str):
            return True  # Allow empty or non-string data to pass through validation

        # Check for XSS patterns
        for pattern in self.xss_patterns:
            if re.search(pattern, data, re.IGNORECASE | re.DOTALL):
                return False

        # Check for SQL injection patterns
        for pattern in self.sql_injection_patterns:
            if re.search(pattern, data, re.IGNORECASE):
                return False

        return True

    def sanitize_input(self, data: str) -> str:
        """
        Sanitize input to prevent attacks
        """
        if not data or not isinstance(data, str):
            return data

        # HTML encode dangerous characters
        sanitized = html.escape(data)

        # URL decode to handle encoded attacks
        try:
            sanitized = urllib.parse.unquote(sanitized)
        except Exception:
            pass  # If URL decode fails, continue with original sanitized string

        return sanitized

    async def validate_request(self, request: Request) -> Dict[str, Any]:
        """
        Validate the incoming request for security threats
        """
        security_issues = []

        # Check content type
        content_type = request.headers.get("content-type", "").lower()
        if content_type and not any(allowed in content_type for allowed in self.allowed_content_types):
            security_issues.append(f"Invalid content type: {content_type}")

        # Get request body if available
        try:
            body = await request.body()
            if body:
                body_str = body.decode('utf-8')
                if not self.validate_input(body_str):
                    security_issues.append("Request body contains potential attack patterns")
        except Exception:
            pass  # If we can't read the body, continue with validation

        # Check query parameters
        for param_name, param_value in request.query_params.items():
            if not self.validate_input(str(param_value)):
                security_issues.append(f"Query parameter '{param_name}' contains potential attack patterns")

        # Check path parameters
        # Note: FastAPI doesn't directly expose path params in middleware,
        # but we can validate the path itself
        if not self.validate_input(request.url.path):
            security_issues.append("URL path contains potential attack patterns")

        # Check headers for potential threats
        for header_name, header_value in request.headers.items():
            if not self.validate_input(str(header_value)):
                security_issues.append(f"Header '{header_name}' contains potential attack patterns")

        return {
            "is_valid": len(security_issues) == 0,
            "issues": security_issues
        }

    def validate_json_input(self, data: Any) -> Dict[str, Any]:
        """
        Validate JSON input data recursively
        """
        issues = []

        def check_recursive(obj, path=""):
            if isinstance(obj, str):
                if not self.validate_input(obj):
                    issues.append(f"String value at '{path}' contains potential attack patterns")
                return self.sanitize_input(obj)
            elif isinstance(obj, dict):
                sanitized = {}
                for key, value in obj.items():
                    new_path = f"{path}.{key}" if path else key
                    sanitized[key] = check_recursive(value, new_path)
                return sanitized
            elif isinstance(obj, list):
                return [check_recursive(item, f"{path}[{i}]") for i, item in enumerate(obj)]
            else:
                return obj

        sanitized_data = check_recursive(data)
        return {
            "is_valid": len(issues) == 0,
            "issues": issues,
            "sanitized_data": sanitized_data
        }


# Global instance
security_middleware = SecurityMiddleware()