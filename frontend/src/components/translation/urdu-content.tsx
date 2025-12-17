import React, { useState, useEffect } from 'react';
import { Box, Typography, Alert, CircularProgress } from '@mui/material';

interface UrduContentProps {
  content: string;
  textbookId: string;
  contentId?: string;
  contentType: string; // "textbook", "chapter", "section", "interactive_element"
  targetLanguage?: string;
  onTranslationComplete?: (translatedContent: string) => void;
}

const UrduContent: React.FC<UrduContentProps> = ({
  content,
  textbookId,
  contentId,
  contentType,
  targetLanguage = 'ur',
  onTranslationComplete
}) => {
  const [translatedContent, setTranslatedContent] = useState<string | null>(null);
  const [isTranslating, setIsTranslating] = useState(false);
  const [translationError, setTranslationError] = useState<string | null>(null);
  const [language, setLanguage] = useState(targetLanguage);

  useEffect(() => {
    if (content) {
      translateContent();
    }
  }, [content, textbookId, contentId, contentType, language]);

  const translateContent = async () => {
    if (!content) return;

    setIsTranslating(true);
    setTranslationError(null);

    try {
      // In a real implementation, this would call the backend API
      // const response = await fetch(`/api/translation/translate`, {
      //   method: 'POST',
      //   headers: { 'Content-Type': 'application/json' },
      //   body: JSON.stringify({
      //     content: content,
      //     textbook_id: textbookId,
      //     content_id: contentId,
      //     content_type: contentType,
      //     target_language: language
      //   })
      // });

      // const result = await response.json();
      // setTranslatedContent(result.translated_content);

      // For now, simulate translation with placeholder text in Arabic script
      const simulatedTranslation = `[${language === 'ur' ? 'اردو ترجمہ' : 'Translated content'}] ${content}`;
      setTranslatedContent(simulatedTranslation);

      if (onTranslationComplete) {
        onTranslationComplete(simulatedTranslation);
      }
    } catch (error) {
      setTranslationError(`Failed to translate content to ${language === 'ur' ? 'Urdu' : language}. Showing original content.`);
      setTranslatedContent(content);
      console.error('Translation error:', error);
    } finally {
      setIsTranslating(false);
    }
  };

  const getLanguageName = (langCode: string) => {
    switch (langCode) {
      case 'ur':
        return 'Urdu';
      case 'es':
        return 'Spanish';
      case 'fr':
        return 'French';
      case 'de':
        return 'German';
      default:
        return langCode.toUpperCase();
    }
  };

  return (
    <Box>
      {isTranslating && (
        <Box sx={{ display: 'flex', alignItems: 'center', mb: 2 }}>
          <CircularProgress size={20} sx={{ mr: 1 }} />
          <Typography variant="body2" color="text.secondary">
            Translating to {getLanguageName(language)}...
          </Typography>
        </Box>
      )}

      {translationError && (
        <Alert severity="warning" sx={{ mb: 2 }}>
          {translationError}
        </Alert>
      )}

      {translatedContent && (
        <Box sx={{
          direction: language === 'ur' ? 'rtl' : 'ltr',
          textAlign: language === 'ur' ? 'right' : 'left'
        }}>
          <Typography
            variant="body1"
            paragraph
            sx={{
              fontFamily: language === 'ur' ? '"Noto Sans Arabic", "Segoe UI", Tahoma, Geneva, Verdana, sans-serif' : 'inherit',
              fontSize: language === 'ur' ? '1.1rem' : 'inherit'
            }}
          >
            {translatedContent}
          </Typography>
          <Typography variant="caption" color="text.secondary">
            {getLanguageName(language)} Translation
          </Typography>
        </Box>
      )}

      {!translatedContent && !isTranslating && (
        <Typography variant="body2" color="text.secondary">
          Translation will appear here after processing.
        </Typography>
      )}
    </Box>
  );
};

export default UrduContent;