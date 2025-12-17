import React, { useEffect, useState } from 'react';
import { Box, LinearProgress, Typography, Alert, Paper } from '@mui/material';

interface ProgressTrackerProps {
  textbookId: string;
  onProgressUpdate?: (progress: number, status: string) => void;
  onComplete?: () => void;
}

const ProgressTracker: React.FC<ProgressTrackerProps> = ({
  textbookId,
  onProgressUpdate,
  onComplete
}) => {
  const [progress, setProgress] = useState<number>(0);
  const [status, setStatus] = useState<string>('Initializing...');
  const [error, setError] = useState<string | null>(null);
  const [isComplete, setIsComplete] = useState<boolean>(false);

  useEffect(() => {
    // Simulate progress updates
    const progressInterval = setInterval(() => {
      setProgress(prev => {
        const newProgress = prev + Math.floor(Math.random() * 10) + 1;
        const cappedProgress = Math.min(newProgress, 100);

        if (cappedProgress >= 100) {
          setIsComplete(true);
          if (onComplete) {
            onComplete();
          }
          clearInterval(progressInterval);
        }

        // Update status based on progress
        let newStatus = '';
        if (cappedProgress < 20) {
          newStatus = 'Initializing content generation...';
        } else if (cappedProgress < 40) {
          newStatus = 'Generating chapter outlines...';
        } else if (cappedProgress < 60) {
          newStatus = 'Creating detailed content...';
        } else if (cappedProgress < 80) {
          newStatus = 'Adding interactive elements...';
        } else if (cappedProgress < 100) {
          newStatus = 'Finalizing textbook structure...';
        } else {
          newStatus = 'Generation complete!';
        }

        setStatus(newStatus);

        if (onProgressUpdate) {
          onProgressUpdate(cappedProgress, newStatus);
        }

        return cappedProgress;
      });
    }, 1000); // Update every second

    return () => {
      clearInterval(progressInterval);
    };
  }, [textbookId, onProgressUpdate, onComplete]);

  return (
    <Box sx={{ p: 3 }}>
      <Paper sx={{ p: 3, mb: 2 }}>
        <Typography variant="h6" gutterBottom>
          Textbook Generation Progress
        </Typography>

        <Typography variant="body2" color="text.secondary" gutterBottom>
          {status}
        </Typography>

        <Box sx={{ display: 'flex', alignItems: 'center', mb: 1 }}>
          <Box sx={{ flex: 1, mr: 2 }}>
            <LinearProgress
              variant="determinate"
              value={progress}
              sx={{
                height: 10,
                borderRadius: 5,
                '& .MuiLinearProgress-bar': {
                  backgroundColor: isComplete ? '#4caf50' : '#1976d2'
                }
              }}
            />
          </Box>
          <Typography variant="body2" color="text.secondary">
            {progress}%
          </Typography>
        </Box>
      </Paper>

      {error && (
        <Alert severity="error" sx={{ mb: 2 }}>
          {error}
        </Alert>
      )}

      <Paper sx={{ p: 2, bgcolor: 'grey.50' }}>
        <Typography variant="body2">
          <strong>Current Task:</strong> {status}
        </Typography>
        <Typography variant="body2" sx={{ mt: 1 }}>
          <strong>Textbook ID:</strong> {textbookId}
        </Typography>
        <Typography variant="body2" sx={{ mt: 1 }}>
          <strong>Estimated Time:</strong> {Math.round((100 - progress) * 0.5)} seconds remaining
        </Typography>
      </Paper>
    </Box>
  );
};

export default ProgressTracker;