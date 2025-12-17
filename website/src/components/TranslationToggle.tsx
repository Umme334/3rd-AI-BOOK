import React, { useState, useEffect } from 'react';
import BrowserOnly from '@docusaurus/BrowserOnly';

interface TranslationToggleProps {
  textbookId: string;
  contentId?: string;
  content?: string;
}

const TranslationToggle: React.FC<TranslationToggleProps> = ({ textbookId, contentId, content }) => {
  const [isUrdu, setIsUrdu] = useState(false);
  const [loading, setLoading] = useState(false);
  const [translatedContent, setTranslatedContent] = useState<string | null>(null);
  const [currentContent, setCurrentContent] = useState(content || '');

  const toggleTranslation = async () => {
    if (isUrdu) {
      // Switch back to original content
      setIsUrdu(false);
      setCurrentContent(content || '');
      setTranslatedContent(null);
      return;
    }

    if (!content) {
      alert('No content to translate');
      return;
    }

    setLoading(true);

    try {
      const response = await fetch('http://localhost:8000/translation/translate', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          content: content,
          textbook_id: textbookId,
          content_id: contentId,
          content_type: 'text',
          target_language: 'ur'
        }),
      });

      if (response.ok) {
        const data = await response.json();
        setTranslatedContent(data.translated_content);
        setCurrentContent(data.translated_content);
        setIsUrdu(true);
      } else {
        throw new Error('Failed to translate content');
      }
    } catch (error) {
      console.error('Error translating content:', error);
      alert('Failed to translate content. Please try again.');
    } finally {
      setLoading(false);
    }
  };

  if (loading) {
    return (
      <div className="translation-toggle loading">
        <span>Translating to Urdu...</span>
      </div>
    );
  }

  return (
    <div className="translation-toggle">
      <button
        className={`translation-button ${isUrdu ? 'urdu' : 'english'}`}
        onClick={toggleTranslation}
        disabled={loading}
      >
        {isUrdu ? 'English' : 'اردو'}
      </button>
      {isUrdu && translatedContent && (
        <div className="translation-info">
          <small>Translated from English</small>
        </div>
      )}
    </div>
  );
};

// Wrapper to ensure component only runs in browser environment
const TranslationToggleWrapper: React.FC<{textbookId: string, contentId?: string, content?: string}> = (props) => {
  return (
    <BrowserOnly fallback={<div>Loading translation...</div>}>
      {() => <TranslationToggle {...props} />}
    </BrowserOnly>
  );
};

export default TranslationToggleWrapper;