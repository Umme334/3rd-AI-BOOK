import React, { useState, useEffect } from 'react';
import { Box, Typography, Chip, Alert } from '@mui/material';

interface ContentAdapterProps {
  content: string;
  contentType: string; // 'textbook', 'chapter', 'section'
  originalContent: string;
  userId?: string;
  onContentAdapted?: (adaptedContent: string) => void;
}

const ContentAdapter: React.FC<ContentAdapterProps> = ({
  content,
  contentType,
  originalContent,
  userId,
  onContentAdapted
}) => {
  const [adaptedContent, setAdaptedContent] = useState(content);
  const [isAdapting, setIsAdapting] = useState(false);
  const [adaptationError, setAdaptationError] = useState<string | null>(null);

  useEffect(() => {
    // If user ID is provided, adapt content based on user profile
    if (userId) {
      adaptContentForUser();
    } else {
      // If no user ID, use original content
      setAdaptedContent(originalContent);
    }
  }, [userId, originalContent]);

  const adaptContentForUser = async () => {
    if (!userId) return;

    setIsAdapting(true);
    setAdaptationError(null);

    try {
      // In a real implementation, this would call the backend API
      // const response = await fetch(`/api/personalization/adapt`, {
      //   method: 'POST',
      //   headers: { 'Content-Type': 'application/json' },
      //   body: JSON.stringify({
      //     content: originalContent,
      //     user_id: userId,
      //     content_type: contentType
      //   })
      // });

      // const result = await response.json();
      // setAdaptedContent(result.adapted_content);

      // For now, simulate adaptation by adding a note about personalization
      const simulatedAdaptation = `${originalContent}\n\n[This content has been adapted based on your technical background: Beginner to Intermediate level]`;
      setAdaptedContent(simulatedAdaptation);

      if (onContentAdapted) {
        onContentAdapted(simulatedAdaptation);
      }
    } catch (error) {
      setAdaptationError('Failed to adapt content for your profile. Showing original content.');
      setAdaptedContent(originalContent);
      console.error('Content adaptation error:', error);
    } finally {
      setIsAdapting(false);
    }
  };

  return (
    <Box>
      {isAdapting && (
        <Alert severity="info" sx={{ mb: 2 }}>
          Adapting content to match your background...
        </Alert>
      )}

      {adaptationError && (
        <Alert severity="warning" sx={{ mb: 2 }}>
          {adaptationError}
        </Alert>
      )}

      <Box sx={{ display: 'flex', alignItems: 'center', mb: 1 }}>
        <Chip
          label={userId ? 'Personalized Content' : 'General Content'}
          color={userId ? 'primary' : 'default'}
          size="small"
          variant={userId ? 'filled' : 'outlined'}
        />
        <Typography variant="caption" sx={{ ml: 1, color: 'text.secondary' }}>
          {contentType}
        </Typography>
      </Box>

      <Typography variant="body1" paragraph>
        {adaptedContent}
      </Typography>
    </Box>
  );
};

export default ContentAdapter;