from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
import os
from dotenv import load_dotenv

from src.api.textbooks import router as textbooks_router
from src.api.auth import router as auth_router
from src.api.personalization import router as personalization_router
from src.api.translation import router as translation_router

# Load environment variables
load_dotenv()

# Create FastAPI app
app = FastAPI(
    title="AI-Native Textbook for Physical AI and Humanoid Robotics API",
    description="API for generating interactive textbooks for Physical AI and Humanoid Robotics with RAG chatbot, personalization, and Urdu translation",
    version="1.0.0"
)

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, replace with specific origins
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Include API routers (excluding chatbot which has vector store dependencies)
app.include_router(textbooks_router)
app.include_router(auth_router)
app.include_router(personalization_router)
app.include_router(translation_router)

@app.get("/")
async def root():
    return {
        "message": "AI-Native Textbook for Physical AI and Humanoid Robotics API",
        "status": "running",
        "features": [
            "Textbook generation",
            "User authentication with background capture",
            "Content personalization",
            "Urdu translation",
            "Docusaurus export for GitHub Pages"
        ]
    }

@app.get("/health")
async def health_check():
    return {"status": "healthy", "timestamp": __import__('datetime').datetime.now()}

# For running with uvicorn
# if __name__ == "__main__":
#     import uvicorn
#     uvicorn.run(app, host="0.0.0.0", port=int(os.getenv("BACKEND_PORT", 8000)))