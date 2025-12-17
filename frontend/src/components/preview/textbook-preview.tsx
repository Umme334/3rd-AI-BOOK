import React from 'react';
import { Box, Typography, Paper, Button, LinearProgress, Alert } from '@mui/material';

interface TextbookPreviewProps {
  textbook: any;
  onExport?: () => void;
  onRegenerate?: () => void;
  isLoading?: boolean;
  error?: string;
}

const TextbookPreview: React.FC<TextbookPreviewProps> = ({
  textbook,
  onExport,
  onRegenerate,
  isLoading = false,
  error
}) => {
  if (isLoading) {
    return (
      <Box sx={{ p: 3, textAlign: 'center' }}>
        <Typography variant="h5" gutterBottom>
          Generating Your Textbook...
        </Typography>
        <LinearProgress sx={{ mb: 2 }} />
        <Typography variant="body1">
          Please wait while we generate your interactive textbook on Physical AI and Humanoid Robotics.
        </Typography>
      </Box>
    );
  }

  if (error) {
    return (
      <Box sx={{ p: 3 }}>
        <Alert severity="error" sx={{ mb: 2 }}>
          {error}
        </Alert>
        {onRegenerate && (
          <Button variant="contained" onClick={onRegenerate}>
            Try Again
          </Button>
        )}
      </Box>
    );
  }

  if (!textbook) {
    return (
      <Box sx={{ p: 3, textAlign: 'center' }}>
        <Typography variant="h5" gutterBottom>
          No Textbook Generated Yet
        </Typography>
        <Typography variant="body1" color="text.secondary">
          Generate a textbook first to see the preview here.
        </Typography>
      </Box>
    );
  }

  return (
    <Box sx={{ p: 3 }}>
      <Box sx={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', mb: 3 }}>
        <Typography variant="h4" gutterBottom>
          {textbook.title || 'Generated Textbook Preview'}
        </Typography>
        {onExport && (
          <Button variant="contained" onClick={onExport}>
            Export Textbook
          </Button>
        )}
      </Box>

      <Paper sx={{ p: 3, mb: 3 }}>
        <Typography variant="h6">Subject:</Typography>
        <Typography variant="body1" gutterBottom>
          {textbook.subject || 'Physical AI and Humanoid Robotics'}
        </Typography>

        <Typography variant="h6">Difficulty:</Typography>
        <Typography variant="body1" gutterBottom>
          {textbook.difficulty || 'Intermediate'}
        </Typography>

        <Typography variant="h6">Target Audience:</Typography>
        <Typography variant="body1" gutterBottom>
          {textbook.targetAudience || 'Students and Researchers'}
        </Typography>

        <Typography variant="h6">Chapters:</Typography>
        <Typography variant="body1">
          {textbook.chapters ? textbook.chapters.length : 0} chapters
        </Typography>
      </Paper>

      {textbook.chapters && textbook.chapters.map((chapter: any, index: number) => (
        <Paper key={chapter.id || index} sx={{ p: 2, mb: 2 }}>
          <Typography variant="h5" gutterBottom>
            {index + 1}. {chapter.title || `Chapter ${index + 1}`}
          </Typography>

          {chapter.sections && chapter.sections.map((section: any, secIndex: number) => (
            <Box key={section.id || secIndex} sx={{ ml: 2, mb: 2 }}>
              <Typography variant="h6">
                {index + 1}.{secIndex + 1} {section.title || `Section ${secIndex + 1}`}
              </Typography>
              <Typography variant="body2" color="text.secondary" paragraph>
                {section.content?.substring(0, 200) || 'Section content will appear here...'}
                {section.content && section.content.length > 200 && '...'}
              </Typography>
            </Box>
          ))}

          {(!chapter.sections || chapter.sections.length === 0) && (
            <Typography variant="body2" color="text.secondary" sx={{ ml: 2 }}>
              This chapter is being generated...
            </Typography>
          )}
        </Paper>
      ))}

      {(!textbook.chapters || textbook.chapters.length === 0) && (
        <Paper sx={{ p: 3, textAlign: 'center' }}>
          <Typography variant="body1" color="text.secondary">
            The textbook content is being generated. Please check back shortly.
          </Typography>
        </Paper>
      )}
    </Box>
  );
};

export default TextbookPreview;