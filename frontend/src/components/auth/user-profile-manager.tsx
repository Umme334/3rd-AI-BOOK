import React, { useState, useEffect } from 'react';
import {
  Box,
  Button,
  Typography,
  Switch,
  FormControlLabel,
  Alert,
  CircularProgress,
  Tabs,
  Tab
} from '@mui/material';
import { useAuth } from '../../contexts/AuthContext';
import { useNavigate } from 'react-router-dom';
import BackgroundQuestions from './background-questions';
import apiClient from '../../services/api-client';

interface UserProfileManagerProps {
  textbookId?: string;
  chapterId?: string;
  onPersonalizationToggle?: (enabled: boolean) => void;
  onTranslationToggle?: (enabled: boolean) => void;
}

interface UserProfileFormData {
  softwareExperience: string;
  hardwareExperience: string;
  programmingLanguages: string;
  hardwarePlatforms: string;
  roboticsExperience: string;
  mathBackground: string;
  primaryGoal: string;
  backgroundQuestions: string;
}

const UserProfileManager: React.FC<UserProfileManagerProps> = ({
  textbookId,
  chapterId,
  onPersonalizationToggle,
  onTranslationToggle
}) => {
  const { user, updateProfile, isLoading: authLoading } = useAuth();
  const [activeTab, setActiveTab] = useState(0);
  const [personalizationEnabled, setPersonalizationEnabled] = useState(false);
  const [translationEnabled, setTranslationEnabled] = useState(false);
  const [loading, setLoading] = useState(false);
  const [successMessage, setSuccessMessage] = useState('');
  const [errorMessage, setErrorMessage] = useState('');
  const navigate = useNavigate();

  useEffect(() => {
    // Load user preferences from profile if available
    if (user) {
      // Check if personalization is enabled based on user preferences
      const pref = user.preferences || {};
      setPersonalizationEnabled(pref.personalization_enabled || false);
      setTranslationEnabled(pref.translation_urdu_enabled || false);
    }
  }, [user]);

  const handlePersonalizationToggle = (event: React.ChangeEvent<HTMLInputElement>) => {
    const enabled = event.target.checked;
    setPersonalizationEnabled(enabled);
    if (onPersonalizationToggle) {
      onPersonalizationToggle(enabled);
    }

    // Update user preferences
    updatePreferences({ personalization_enabled: enabled });
  };

  const handleTranslationToggle = (event: React.ChangeEvent<HTMLInputElement>) => {
    const enabled = event.target.checked;
    setTranslationEnabled(enabled);
    if (onTranslationToggle) {
      onTranslationToggle(enabled);
    }

    // Update user preferences
    updatePreferences({ translation_urdu_enabled: enabled });
  };

  const updatePreferences = async (newPreferences: Record<string, any>) => {
    if (!user) return;

    try {
      setLoading(true);
      setErrorMessage('');

      const updatedPreferences = {
        ...user.preferences,
        ...newPreferences
      };

      await updateProfile({ preferences: updatedPreferences });
      setSuccessMessage('Preferences updated successfully!');

      setTimeout(() => setSuccessMessage(''), 3000);
    } catch (error) {
      console.error('Error updating preferences:', error);
      setErrorMessage('Failed to update preferences. Please try again.');
    } finally {
      setLoading(false);
    }
  };

  const handleBackgroundUpdate = async (data: UserProfileFormData) => {
    if (!user) return;

    try {
      setLoading(true);
      setErrorMessage('');

      // Transform the form data to match the user profile structure
      const profileUpdates = {
        softwareExperience: data.softwareExperience,
        hardwareExperience: data.hardwareExperience,
        programmingLanguages: data.programmingLanguages.split(',').map(s => s.trim()).filter(s => s),
        hardwarePlatforms: data.hardwarePlatforms.split(',').map(s => s.trim()).filter(s => s),
        roboticsExperience: data.roboticsExperience,
        mathBackground: data.mathBackground,
        primaryGoal: data.primaryGoal,
        backgroundQuestions: data.backgroundQuestions
      };

      await updateProfile(profileUpdates);
      setSuccessMessage('Profile updated successfully!');

      setTimeout(() => setSuccessMessage(''), 3000);
    } catch (error) {
      console.error('Error updating profile:', error);
      setErrorMessage('Failed to update profile. Please try again.');
    } finally {
      setLoading(false);
    }
  };

  const handleTranslateChapter = async () => {
    if (!user || !textbookId || !chapterId) {
      setErrorMessage('Missing required information for translation');
      return;
    }

    try {
      setLoading(true);
      setErrorMessage('');

      // Call the translation API to translate the chapter to Urdu
      const translationRequest = {
        content: "", // In a real implementation, this would be the chapter content
        textbook_id: textbookId,
        content_id: chapterId,
        content_type: "chapter",
        target_language: "ur"
      };

      const result = await apiClient.translateContent(translationRequest);

      // In a real implementation, you would handle the translated content
      // For now, we'll just show a success message
      setSuccessMessage('Chapter translation initiated successfully!');

      setTimeout(() => setSuccessMessage(''), 3000);
    } catch (error) {
      console.error('Error translating chapter:', error);
      setErrorMessage('Failed to translate chapter. Please try again.');
    } finally {
      setLoading(false);
    }
  };

  const handlePersonalizeChapter = async () => {
    if (!user || !textbookId || !chapterId) {
      setErrorMessage('Missing required information for personalization');
      return;
    }

    try {
      setLoading(true);
      setErrorMessage('');

      // Call the personalization API to adapt the chapter content
      const personalizationRequest = {
        content: "", // In a real implementation, this would be the chapter content
        user_id: user.id,
        content_type: "chapter"
      };

      const result = await apiClient.adaptContent(personalizationRequest);

      // In a real implementation, you would handle the personalized content
      // For now, we'll just show a success message
      setSuccessMessage('Chapter personalization initiated successfully!');

      setTimeout(() => setSuccessMessage(''), 3000);
    } catch (error) {
      console.error('Error personalizing chapter:', error);
      setErrorMessage('Failed to personalize chapter. Please try again.');
    } finally {
      setLoading(false);
    }
  };

  const handleSignOut = async () => {
    try {
      setLoading(true);
      setErrorMessage('');

      // Call the backend signout endpoint
      const response = await fetch('http://localhost:8000/auth/signout', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          'Authorization': `Bearer ${localStorage.getItem('better-auth-session') || ''}`
        }
      });

      if (response.ok) {
        // Clear the local session
        localStorage.removeItem('better-auth-session');

        // Navigate to home or login page
        navigate('/');

        setSuccessMessage('Successfully signed out!');
        setTimeout(() => setSuccessMessage(''), 3000);
      } else {
        const data = await response.json();
        throw new Error(data.detail || 'Failed to sign out');
      }
    } catch (error) {
      console.error('Error signing out:', error);
      setErrorMessage('Failed to sign out. Please try again.');
    } finally {
      setLoading(false);
    }
  };

  if (authLoading) {
    return (
      <Box sx={{ display: 'flex', justifyContent: 'center', alignItems: 'center', p: 3 }}>
        <CircularProgress />
      </Box>
    );
  }

  if (!user) {
    return (
      <Box sx={{ p: 3, textAlign: 'center' }}>
        <Typography variant="h6" gutterBottom>
          Please sign in to access personalization features
        </Typography>
        <Button
          variant="contained"
          onClick={() => navigate('/login')}
          sx={{ mt: 2 }}
        >
          Sign In
        </Button>
      </Box>
    );
  }

  return (
    <Box sx={{ p: 3 }}>
      <Box sx={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', mb: 2 }}>
        <Typography variant="h5">
          User Profile & Preferences
        </Typography>
        <Button
          variant="outlined"
          color="secondary"
          onClick={handleSignOut}
          sx={{ ml: 2 }}
        >
          Sign Out
        </Button>
      </Box>

      {successMessage && (
        <Alert severity="success" sx={{ mb: 2 }}>
          {successMessage}
        </Alert>
      )}

      {errorMessage && (
        <Alert severity="error" sx={{ mb: 2 }}>
          {errorMessage}
        </Alert>
      )}

      <Tabs
        value={activeTab}
        onChange={(e, newValue) => setActiveTab(newValue)}
        sx={{ mb: 3 }}
      >
        <Tab label="Preferences" />
        <Tab label="Background Info" />
        <Tab label="Content Features" />
      </Tabs>

      {activeTab === 0 && (
        <Box>
          <Typography variant="h6" gutterBottom>
            Content Preferences
          </Typography>

          <FormControlLabel
            control={
              <Switch
                checked={personalizationEnabled}
                onChange={handlePersonalizationToggle}
                disabled={loading}
              />
            }
            label="Enable Content Personalization"
            sx={{ mb: 2 }}
          />

          <FormControlLabel
            control={
              <Switch
                checked={translationEnabled}
                onChange={handleTranslationToggle}
                disabled={loading}
              />
            }
            label="Enable Urdu Translation"
            sx={{ mb: 2 }}
          />

          <Typography variant="body2" color="text.secondary" sx={{ mt: 2 }}>
            Personalization adapts content difficulty and examples based on your background.
            Urdu translation makes content accessible in your preferred language.
          </Typography>
        </Box>
      )}

      {activeTab === 1 && (
        <BackgroundQuestions
          initialData={{
            softwareExperience: user.softwareExperience || 'intermediate',
            hardwareExperience: user.hardwareExperience || 'beginner',
            programmingLanguages: user.programmingLanguages?.join(', ') || '',
            hardwarePlatforms: user.hardwarePlatforms?.join(', ') || '',
            roboticsExperience: user.roboticsExperience || 'basic',
            mathBackground: user.mathBackground || 'intermediate',
            primaryGoal: user.primaryGoal || '',
            backgroundQuestions: user.backgroundQuestions || ''
          }}
          onSubmit={handleBackgroundUpdate}
        />
      )}

      {activeTab === 2 && (
        <Box>
          <Typography variant="h6" gutterBottom>
            Chapter-Specific Features
          </Typography>

          {textbookId && chapterId ? (
            <>
              <Button
                variant="outlined"
                onClick={handlePersonalizeChapter}
                disabled={loading || !personalizationEnabled}
                sx={{ mr: 2, mb: 2 }}
              >
                Personalize This Chapter
              </Button>

              <Button
                variant="outlined"
                onClick={handleTranslateChapter}
                disabled={loading || !translationEnabled}
                sx={{ mb: 2 }}
              >
                Translate to Urdu
              </Button>

              <Typography variant="body2" color="text.secondary" sx={{ mt: 2 }}>
                {personalizationEnabled
                  ? "Content will be adapted based on your background and preferences."
                  : "Enable personalization in preferences to customize content difficulty."}
              </Typography>

              <Typography variant="body2" color="text.secondary">
                {translationEnabled
                  ? "Content will be available in Urdu for better understanding."
                  : "Enable Urdu translation in preferences to access content in Urdu."}
              </Typography>
            </>
          ) : (
            <Typography variant="body1" color="text.secondary">
              These features are available when viewing specific textbook chapters.
            </Typography>
          )}
        </Box>
      )}

      {loading && (
        <Box sx={{
          position: 'absolute',
          top: 0,
          left: 0,
          right: 0,
          bottom: 0,
          backgroundColor: 'rgba(255, 255, 255, 0.7)',
          display: 'flex',
          justifyContent: 'center',
          alignItems: 'center',
          zIndex: 1
        }}>
          <CircularProgress />
        </Box>
      )}
    </Box>
  );
};

export default UserProfileManager;