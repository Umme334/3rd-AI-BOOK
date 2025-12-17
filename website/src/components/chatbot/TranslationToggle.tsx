import React, { useState, useEffect } from 'react';
import BrowserOnly from '@docusaurus/BrowserOnly';

interface TranslationToggleProps {
  contentId: string; // ID of the content to translate
  originalContent: string; // Original English content
  contentType?: 'text' | 'chapter' | 'section'; // Type of content being translated
  userId?: string;
  onTranslationChange?: (translatedContent: string, language: string) => void;
}

const TranslationToggle: React.FC<TranslationToggleProps> = ({
  contentId,
  originalContent,
  contentType = 'text',
  userId,
  onTranslationChange
}) => {
  const [isTranslated, setIsTranslated] = useState(false);
  const [translatedContent, setTranslatedContent] = useState('');
  const [loading, setLoading] = useState(false);
  const [language, setLanguage] = useState('ur'); // Default to Urdu

  const toggleTranslation = async () => {
    if (!userId) {
      alert('Please sign in to use translation features');
      return;
    }

    if (isTranslated) {
      // Switch back to original content
      setIsTranslated(false);
      setTranslatedContent('');
      if (onTranslationChange) {
        onTranslationChange(originalContent, 'en');
      }
      return;
    }

    setLoading(true);

    try {
      const response = await fetch('/api/translation/translate', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          content_id: contentId,
          content: originalContent,
          target_language: language,
          content_type: contentType,
          user_id: userId
        }),
      });

      if (response.ok) {
        const data = await response.json();
        setTranslatedContent(data.translated_content);
        setIsTranslated(true);
        if (onTranslationChange) {
          onTranslationChange(data.translated_content, language);
        }
      } else {
        const errorData = await response.json();
        throw new Error(errorData.detail || 'Failed to translate content');
      }
    } catch (error) {
      console.error('Error translating content:', error);
      alert(`Failed to translate content: ${error.message}`);
    } finally {
      setLoading(false);
    }
  };

  return (
    <div className="translation-toggle">
      <button
        onClick={toggleTranslation}
        disabled={loading || !userId}
        className={`translation-btn ${isTranslated ? 'translated' : ''}`}
        title={isTranslated ? 'Switch to English' : 'Translate to Urdu'}
      >
        {loading ? (
          'Translating...'
        ) : isTranslated ? (
          '🇬🇧 English'
        ) : (
          '🇵🇰 Urdu'
        )}
      </button>
      {!userId && (
        <span className="auth-hint">
          Sign in to enable translation
        </span>
      )}
      {isTranslated && translatedContent && (
        <div className="translation-content">
          <div className="original-content" style={{ display: 'none' }}>
            {originalContent}
          </div>
        </div>
      )}
    </div>
  );
};

// Wrapper to ensure component only runs in browser environment
const TranslationToggleWrapper: React.FC<TranslationToggleProps> = (props) => {
  return (
    <BrowserOnly fallback={<div>Loading translation...</div>}>
      {() => <TranslationToggle {...props} />}
    </BrowserOnly>
  );
};

export default TranslationToggleWrapper;