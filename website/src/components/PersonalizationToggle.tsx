import React, { useState, useEffect } from 'react';
import BrowserOnly from '@docusaurus/BrowserOnly';

interface PersonalizationToggleProps {
  userId?: string;
  textbookId: string;
}

const PersonalizationToggle: React.FC<PersonalizationToggleProps> = ({ userId, textbookId }) => {
  const [isPersonalized, setIsPersonalized] = useState(false);
  const [loading, setLoading] = useState(false);
  const [userProfile, setUserProfile] = useState<any>(null);

  // Load user profile on component mount
  useEffect(() => {
    if (userId) {
      loadUserProfile();
    }
  }, [userId]);

  const loadUserProfile = async () => {
    if (!userId) return;

    try {
      setLoading(true);
      const response = await fetch(`http://localhost:8000/auth/profile/${userId}`);
      if (response.ok) {
        const data = await response.json();
        setUserProfile(data);
      }
    } catch (error) {
      console.error('Error loading user profile:', error);
    } finally {
      setLoading(false);
    }
  };

  const togglePersonalization = async () => {
    if (!userId) {
      alert('Please log in to enable personalization');
      return;
    }

    setLoading(true);

    try {
      const response = await fetch('http://localhost:8000/personalization/toggle', {
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
        setIsPersonalized(!isPersonalized);
      } else {
        throw new Error('Failed to toggle personalization');
      }
    } catch (error) {
      console.error('Error toggling personalization:', error);
      alert('Failed to toggle personalization. Please try again.');
    } finally {
      setLoading(false);
    }
  };

  if (loading) {
    return (
      <div className="personalization-toggle loading">
        <span>Loading personalization settings...</span>
      </div>
    );
  }

  return (
    <div className="personalization-toggle">
      <label className="personalization-label">
        <input
          type="checkbox"
          checked={isPersonalized}
          onChange={togglePersonalization}
          disabled={loading || !userId}
        />
        <span className="personalization-text">
          {isPersonalized ? 'Personalized Content' : 'Standard Content'}
        </span>
      </label>
      {userProfile && (
        <div className="user-profile-preview">
          <small>Based on your profile: {userProfile.software_experience} background</small>
        </div>
      )}
    </div>
  );
};

// Wrapper to ensure component only runs in browser environment
const PersonalizationToggleWrapper: React.FC<{userId?: string, textbookId: string}> = (props) => {
  return (
    <BrowserOnly fallback={<div>Loading personalization...</div>}>
      {() => <PersonalizationToggle {...props} />}
    </BrowserOnly>
  );
};

export default PersonalizationToggleWrapper;