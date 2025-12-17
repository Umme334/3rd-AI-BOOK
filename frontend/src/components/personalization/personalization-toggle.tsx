import React, { useState } from 'react';

interface PersonalizationToggleProps {
  textbookId: string;
  contentId?: string;
  contentType: string; // "textbook", "chapter", "section"
  onToggle: (enabled: boolean) => void;
}

const PersonalizationToggle: React.FC<PersonalizationToggleProps> = ({
  textbookId,
  contentId,
  contentType,
  onToggle
}) => {
  const [isEnabled, setIsEnabled] = useState(false);

  const handleToggle = () => {
    const newEnabledState = !isEnabled;
    setIsEnabled(newEnabledState);
    onToggle(newEnabledState);
  };

  return (
    <div className="personalization-toggle">
      <label className="toggle-switch">
        <input
          type="checkbox"
          checked={isEnabled}
          onChange={handleToggle}
        />
        <span className="toggle-slider"></span>
      </label>
      <span className="toggle-label">
        Personalize Content
      </span>
      {isEnabled && (
        <span className="toggle-info">
          Content will be adapted to your background
        </span>
      )}
    </div>
  );
};

export default PersonalizationToggle;