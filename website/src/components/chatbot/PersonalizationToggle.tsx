import React, { useState, useEffect } from 'react';
import BrowserOnly from '@docusaurus/BrowserOnly';

interface PersonalizationToggleProps {
  textbookId: string;
  userId?: string;
  onPersonalizationChange?: (enabled: boolean) => void;
}

const PersonalizationToggle: React.FC<PersonalizationToggleProps> = ({
  textbookId,
  userId,
  onPersonalizationChange
}) => {
  const [isPersonalized, setIsPersonalized] = useState(false);
  const [loading, setLoading] = useState(false);
  const [userProfile, setUserProfile] = useState<any>(null);

  // Check if personalization is enabled for this user
  useEffect(() => {
    if (userId) {
      checkPersonalizationStatus();
    }
  }, [userId, textbookId]);

  const checkPersonalizationStatus = async () => {
    if (!userId) return;

    try {
      const response = await fetch(`/api/personalization/status?user_id=${userId}&textbook_id=${textbookId}`);
      if (response.ok) {
        const data = await response.json();
        setIsPersonalized(data.enabled);
      }
    } catch (error) {
      console.error('Error checking personalization status:', error);
    }
  };

  const togglePersonalization = async () => {
    if (!userId) {
      alert('Please sign in to use personalization features');
      return;
    }

    setLoading(true);

    try {
      const response = await fetch('/api/personalization/toggle', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          user_id: userId,
          textbook_id: textbookId,
          enabled: !isPersonalized
        }),
      });

      if (response.ok) {
        const data = await response.json();
        setIsPersonalized(data.enabled);
        if (onPersonalizationChange) {
          onPersonalizationChange(data.enabled);
        }
      } else {
        throw new Error('Failed to toggle personalization');
      }
    } catch (error) {
      console.error('Error toggling personalization:', error);
      alert('Failed to update personalization settings');
    } finally {
      setLoading(false);
    }
  };

  return (
    <div className="personalization-toggle">
      <label className="toggle-switch">
        <input
          type="checkbox"
          checked={isPersonalized}
          onChange={togglePersonalization}
          disabled={loading || !userId}
        />
        <span className="toggle-slider"></span>
      </label>
      <span className="toggle-label">
        {loading ? 'Updating...' : isPersonalized ? 'Personalized Content' : 'Standard Content'}
      </span>
      {!userId && (
        <span className="auth-hint">
          Sign in to enable personalization
        </span>
      )}
    </div>
  );
};

// Wrapper to ensure component only runs in browser environment
const PersonalizationToggleWrapper: React.FC<PersonalizationToggleProps> = (props) => {
  return (
    <BrowserOnly fallback={<div>Loading personalization...</div>}>
      {() => <PersonalizationToggle {...props} />}
    </BrowserOnly>
  );
};

export default PersonalizationToggleWrapper;