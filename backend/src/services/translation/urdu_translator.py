from typing import Dict, Any, Optional
import logging
from google.cloud import translate
from google.oauth2 import service_account
import os
from pathlib import Path


logger = logging.getLogger(__name__)


class UrduTranslator:
    """
    Service for translating content to Urdu using Google Cloud Translation API.
    """

    def __init__(self):
        # Initialize Google Cloud Translation API client
        credentials_path = os.path.join(Path(__file__).parent, "..", "..", "..", "..", "google-service-account-key.json")

        try:
            if os.path.exists(credentials_path):
                credentials = service_account.Credentials.from_service_account_file(credentials_path)
                self.client = translate.TranslationServiceClient(credentials=credentials)
                self.project_id = credentials.project_id
            else:
                # Fallback to environment-based authentication
                self.client = translate.TranslationServiceClient()
                self.project_id = os.getenv("GOOGLE_CLOUD_PROJECT")
        except Exception as e:
            logger.warning(f"Could not initialize Google Cloud Translation: {e}")
            # Set to None to indicate that the real service is not available
            self.client = None
            self.project_id = None

    async def translate(
        self,
        content: str,
        source_language: str = "en",
        translation_level: str = "literal"
    ) -> str:
        """
        Translate content to Urdu.

        Args:
            content: The content to translate
            source_language: The source language code (default: 'en' for English)
            translation_level: The level of translation ('literal', 'contextual', 'adaptive')

        Returns:
            The translated content in Urdu
        """
        try:
            # For now, we'll use a simple translation approach
            # In a real implementation, we would have different translation levels
            if translation_level == "literal":
                translated_content = self._translate_literal(content, source_language)
            elif translation_level == "contextual":
                translated_content = self._translate_contextual(content, source_language)
            else:  # adaptive or default
                translated_content = self._translate_adaptive(content, source_language)

            return translated_content

        except Exception as e:
            logger.error(f"Error translating to Urdu: {str(e)}")
            # Return original content with error note if translation fails
            return f"[TRANSLATION ERROR: {str(e)}]\n\n{content}"

    def _translate_literal(self, content: str, source_language: str) -> str:
        """
        Perform literal translation of content to Urdu.
        """
        try:
            if self.client is not None and self.project_id:
                # Use Google Cloud Translation API v3
                location = "global"  # or "us-central1" for regional
                parent = f"projects/{self.project_id}/locations/{location}"

                response = self.client.translate_text(
                    request={
                        "parent": parent,
                        "contents": [content],
                        "mime_type": "text/plain",
                        "source_language_code": source_language,
                        "target_language_code": 'ur'
                    }
                )
                return response.translations[0].translated_text
            else:
                # Fallback if no client is available
                logger.warning("Google Cloud Translation not available, using mock translation")
                return f"[(urdu: {content})]"  # Simple mock translation
        except Exception as e:
            logger.error(f"Literal translation failed: {str(e)}")
            # Fallback: return content with note
            return f"[(urdu: translation unavailable)] {content}"

    def _translate_contextual(self, content: str, source_language: str) -> str:
        """
        Perform contextual translation of content to Urdu, considering domain-specific terms.
        """
        try:
            # For Physical AI and Humanoid Robotics content, we may want to handle
            # specific terminology differently
            # This is a simplified implementation - in a real system, we'd have
            # a terminology database for domain-specific translations

            # First perform standard translation
            translated = self._translate_literal(content, source_language)

            # Then handle domain-specific terms if needed
            # (This would be expanded in a full implementation)

            return translated
        except Exception as e:
            logger.error(f"Contextual translation failed: {str(e)}")
            return self._translate_literal(content, source_language)

    def _translate_adaptive(self, content: str, source_language: str) -> str:
        """
        Perform adaptive translation that considers user preferences and context.
        """
        try:
            # Adaptive translation would consider various factors like:
            # - User's technical background
            # - Context of the content (textbook, chapter, section)
            # - Previous translation preferences
            # For now, we'll just use the literal translation
            return self._translate_literal(content, source_language)
        except Exception as e:
            logger.error(f"Adaptive translation failed: {str(e)}")
            return self._translate_literal(content, source_language)

    def is_urdu_text(self, text: str) -> bool:
        """
        Check if the given text is in Urdu (contains Urdu characters).
        """
        # Urdu uses Arabic script, so we check for Arabic/Urdu Unicode ranges
        for char in text:
            if '\u0600' <= char <= '\u06FF' or '\u0750' <= char <= '\u077F':
                return True
        return False

    def get_urdu_text_length(self, text: str) -> int:
        """
        Get the length of Urdu text, accounting for multi-byte characters.
        """
        return len(text.encode('utf-8'))

    async def translate_batch(
        self,
        contents: list[str],
        source_language: str = "en"
    ) -> list[str]:
        """
        Translate multiple pieces of content to Urdu in batch.
        """
        translated_contents = []
        for content in contents:
            translated = await self.translate(content, source_language)
            translated_contents.append(translated)
        return translated_contents

    async def detect_language(self, content: str) -> str:
        """
        Detect the source language of the content.
        """
        try:
            if self.client is not None and self.project_id:
                # Use Google Cloud Translation API v3 to detect language
                location = "global"
                parent = f"projects/{self.project_id}/locations/{location}"

                response = self.client.detect_language(
                    request={
                        "parent": parent,
                        "content": content,
                        "mime_type": "text/plain"
                    }
                )

                if response.languages:
                    return response.languages[0].language_code
                else:
                    return "en"  # Default to English if no language detected
            else:
                # Fallback if no client is available
                logger.warning("Google Cloud Translation not available for language detection, defaulting to English")
                return "en"
        except Exception as e:
            logger.error(f"Language detection failed: {str(e)}")
            return "en"  # Default to English