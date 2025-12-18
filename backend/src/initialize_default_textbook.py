"""
Initialization script to create a default textbook if it doesn't exist.
This ensures the chatbot has a valid textbook to work with.
"""
import json
import uuid
from datetime import datetime
from pathlib import Path

def initialize_default_textbook():
    """Create a default textbook if it doesn't exist."""
    # Define the default textbook ID
    textbook_id = "physical-ai-textbook"

    # Define the base path for textbooks
    base_path = Path("docs/textbooks")
    base_path.mkdir(parents=True, exist_ok=True)

    # Define the textbook file path
    textbook_file = base_path / f"{textbook_id}.json"

    # If the textbook doesn't exist, create a basic one
    if not textbook_file.exists():
        print(f"Creating default textbook: {textbook_id}")

        # Create a basic textbook structure
        default_textbook = {
            "id": textbook_id,
            "title": "Physical AI & Humanoid Robotics",
            "subject": "Physical AI and Humanoid Robotics",
            "difficulty": "intermediate",
            "target_audience": "Students and Researchers",
            "chapters": [],
            "metadata": {
                "created_at": datetime.now().isoformat(),
                "updated_at": datetime.now().isoformat(),
                "description": "Default textbook for Physical AI and Humanoid Robotics assistant"
            },
            "export_formats": ["pdf", "html", "md"],
            "status": "complete",  # Set as complete so chatbot can use it
            "rag_indexed": True,  # Mark as indexed for RAG
            "created_at": datetime.now().isoformat(),
            "updated_at": datetime.now().isoformat()
        }

        # Save the textbook to file
        with open(textbook_file, 'w', encoding='utf-8') as f:
            json.dump(default_textbook, f, indent=2, ensure_ascii=False, default=str)

        print(f"Default textbook '{textbook_id}' created successfully.")
    else:
        print(f"Default textbook '{textbook_id}' already exists.")

if __name__ == "__main__":
    initialize_default_textbook()