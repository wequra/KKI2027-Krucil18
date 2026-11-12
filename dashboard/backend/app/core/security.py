from fastapi import Security, HTTPException, status
from fastapi.security import APIKeyHeader
from app.core.config import settings

api_key_header = APIKeyHeader(name="X-API-Key", auto_error=False)

async def verify_jetson_api_key(api_key: str = Security(api_key_header)):
    if api_key is None or api_key != settings.JETSON_API_KEY:
        raise HTTPException(
            status_code=status.HTTP_403_FORBIDDEN, 
            detail="Invalid or missing API Key. Access denied."
        )