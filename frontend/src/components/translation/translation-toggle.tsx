import React, { useState } from 'react';

interface TranslationToggleProps {
  textbookId: string;
  contentId?: string;
  contentType: string; // "textbook", "chapter", "section", "interactive_element"
  onToggle: (enabled: boolean, language?: string) => void;
}

const TranslationToggle: React.FC<TranslationToggleProps> = ({
  textbookId,
  contentId,
  contentType,
  onToggle
}) => {
  const [isTranslated, setIsTranslated] = useState(false);
  const [targetLanguage, setTargetLanguage] = useState('ur'); // Default to Urdu

  const handleToggle = () => {
    const newTranslatedState = !isTranslated;
    setIsTranslated(newTranslatedState);
    onToggle(newTranslatedState, targetLanguage);
  };

  const handleLanguageChange = (e: React.ChangeEvent<HTMLSelectElement>) => {
    const newLanguage = e.target.value;
    setTargetLanguage(newLanguage);

    // If already enabled, update with new language
    if (isTranslated) {
      onToggle(true, newLanguage);
    }
  };

  return (
    <div className="translation-toggle">
      <label className="toggle-switch">
        <input
          type="checkbox"
          checked={isTranslated}
          onChange={handleToggle}
        />
        <span className="toggle-slider"></span>
      </label>
      <span className="toggle-label">
        Translate Content
      </span>
      <select
        value={targetLanguage}
        onChange={handleLanguageChange}
        disabled={!isTranslated}
      >
        <option value="ur">Urdu</option>
        <option value="es">Spanish</option>
        <option value="fr">French</option>
        <option value="de">German</option>
      </select>
      {isTranslated && (
        <span className="toggle-info">
          Content translated to {targetLanguage === 'ur' ? 'Urdu' : targetLanguage.toUpperCase()}
        </span>
      )}
    </div>
  );
};

export default TranslationToggle;