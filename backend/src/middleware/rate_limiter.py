from fastapi import HTTPException, Request
from typing import Dict
from datetime import datetime, timedelta
import time
from collections import defaultdict
import asyncio
from functools import wraps


class RateLimiter:
    """
    Rate limiting middleware for API requests
    """

    def __init__(self, requests_per_minute: int = 60):
        self.requests_per_minute = requests_per_minute
        self.requests: Dict[str, list] = defaultdict(list)

    def is_allowed(self, identifier: str) -> bool:
        """
        Check if the request is allowed based on rate limits
        """
        now = datetime.now()
        # Remove requests older than 1 minute
        self.requests[identifier] = [
            req_time for req_time in self.requests[identifier]
            if now - req_time < timedelta(minutes=1)
        ]

        # Check if we're under the limit
        if len(self.requests[identifier]) < self.requests_per_minute:
            self.requests[identifier].append(now)
            return True

        return False

    async def check_rate_limit(self, request: Request) -> bool:
        """
        Check rate limit for the incoming request
        """
        # Get client IP for rate limiting
        client_ip = request.client.host
        user_id = request.headers.get("X-User-ID")  # Optional user ID header

        # Use user ID if available, otherwise use IP
        identifier = user_id if user_id else client_ip

        return self.is_allowed(identifier)


# Global instance with default rate limit
rate_limiter = RateLimiter(requests_per_minute=60)


def rate_limit(max_requests: int = 60, window_minutes: int = 1):
    """
    Decorator for rate limiting specific endpoints
    """
    limiter = RateLimiter(requests_per_minute=max_requests)

    def decorator(func):
        @wraps(func)
        async def wrapper(*args, **kwargs):
            # In a real implementation, we'd have access to the request here
            # For now, we'll just check against a default identifier
            if not limiter.is_allowed("default"):
                raise HTTPException(
                    status_code=429,
                    detail="Rate limit exceeded. Please try again later."
                )
            return await func(*args, **kwargs)
        return wrapper
    return decorator