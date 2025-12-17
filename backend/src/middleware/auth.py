from fastapi import HTTPException, Request
from fastapi.security import HTTPBearer, HTTPAuthorizationCredentials
from typing import Optional
import jwt
from datetime import datetime, timedelta
import os
from ..config import settings
from ..exceptions import AuthenticationError


class AuthMiddleware:
    """
    Authentication middleware for API requests
    """

    def __init__(self):
        self.secret_key = os.getenv("JWT_SECRET_KEY", settings.jwt_secret_key)
        self.algorithm = os.getenv("JWT_ALGORITHM", "HS256")
        self.security = HTTPBearer()

    async def verify_token(self, token: str) -> dict:
        """
        Verify JWT token and return user payload
        """
        try:
            payload = jwt.decode(token, self.secret_key, algorithms=[self.algorithm])
            return payload
        except jwt.ExpiredSignatureError:
            raise AuthenticationError("Token has expired")
        except jwt.InvalidTokenError:
            raise AuthenticationError("Invalid token")

    async def authenticate_request(self, request: Request) -> Optional[dict]:
        """
        Authenticate the incoming request
        """
        # Extract token from Authorization header
        auth_header = request.headers.get("Authorization")
        if not auth_header or not auth_header.startswith("Bearer "):
            return None

        token = auth_header.split(" ")[1]
        user_payload = await self.verify_token(token)
        return user_payload


# Global instance
auth_middleware = AuthMiddleware()