import os
import json
from typing import Any, Dict, Optional
from pathlib import Path


class FileStorage:
    """
    File-based storage utility for generated textbooks and related content.
    """

    def __init__(self, base_path: str = "docs/textbooks"):
        self.base_path = Path(base_path)
        self.base_path.mkdir(parents=True, exist_ok=True)

    def save_textbook(self, textbook_id: str, content: Dict[str, Any]) -> str:
        """
        Save textbook content to a JSON file.

        Args:
            textbook_id: Unique identifier for the textbook
            content: Dictionary containing textbook data

        Returns:
            Path to the saved file
        """
        file_path = self.base_path / f"{textbook_id}.json"
        with open(file_path, 'w', encoding='utf-8') as f:
            json.dump(content, f, indent=2, ensure_ascii=False, default=str)
        return str(file_path)

    def load_textbook(self, textbook_id: str) -> Optional[Dict[str, Any]]:
        """
        Load textbook content from a JSON file.

        Args:
            textbook_id: Unique identifier for the textbook

        Returns:
            Dictionary containing textbook data or None if not found
        """
        file_path = self.base_path / f"{textbook_id}.json"
        if file_path.exists():
            with open(file_path, 'r', encoding='utf-8') as f:
                return json.load(f)
        return None

    def delete_textbook(self, textbook_id: str) -> bool:
        """
        Delete textbook file.

        Args:
            textbook_id: Unique identifier for the textbook

        Returns:
            True if file was deleted, False if not found
        """
        file_path = self.base_path / f"{textbook_id}.json"
        if file_path.exists():
            file_path.unlink()
            return True
        return False

    def list_textbooks(self) -> list:
        """
        List all textbook files in storage.

        Returns:
            List of textbook IDs
        """
        textbook_files = list(self.base_path.glob("*.json"))
        return [f.stem for f in textbook_files]

    def save_file(self, filename: str, content: str, subdirectory: Optional[str] = None) -> str:
        """
        Save arbitrary content to a file.

        Args:
            filename: Name of the file to save
            content: Content to save
            subdirectory: Optional subdirectory to save in

        Returns:
            Path to the saved file
        """
        if subdirectory:
            save_path = self.base_path / subdirectory
            save_path.mkdir(parents=True, exist_ok=True)
            file_path = save_path / filename
        else:
            file_path = self.base_path / filename

        with open(file_path, 'w', encoding='utf-8') as f:
            f.write(content)
        return str(file_path)