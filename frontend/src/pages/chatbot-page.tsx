import React, { useState } from 'react';
import { Container, Paper, Box, Typography, Button, Dialog, DialogTitle, DialogContent, DialogActions } from '@mui/material';
import { ChatbotWidget } from '../components/chatbot/chatbot-widget';

interface ChatbotPageProps {
  textbookId?: string;
}

const ChatbotPage: React.FC<ChatbotPageProps> = ({ textbookId = 'default-textbook' }) => {
  const [sessionId, setSessionId] = useState<string | null>(null);
  const [showSessionDialog, setShowSessionDialog] = useState(false);

  const handleNewSession = () => {
    const newSessionId = `session-${Date.now()}`;
    setSessionId(newSessionId);
    setShowSessionDialog(false);
  };

  const handleStartNewChat = () => {
    setShowSessionDialog(true);
  };

  return (
    <Container maxWidth="lg" sx={{ py: 3 }}>
      <Paper sx={{ p: 3 }}>
        <Box sx={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', mb: 3 }}>
          <Typography variant="h4">
            Physical AI & Humanoid Robotics Assistant
          </Typography>
          <Button variant="outlined" onClick={handleStartNewChat}>
            New Chat Session
          </Button>
        </Box>

        <Typography variant="body1" sx={{ mb: 2, color: 'text.secondary' }}>
          Ask questions about your textbook content. The AI assistant will provide answers based on the Physical AI and Humanoid Robotics material.
        </Typography>

        <Box sx={{ height: '650px' }}>
          <ChatbotWidget
            textbookId={textbookId}
            sessionId={sessionId || undefined}
          />
        </Box>
      </Paper>

      <Dialog open={showSessionDialog} onClose={() => setShowSessionDialog(false)}>
        <DialogTitle>Start New Chat Session</DialogTitle>
        <DialogContent>
          <Typography variant="body1" sx={{ mt: 1 }}>
            Starting a new chat session will clear your current conversation history.
          </Typography>
        </DialogContent>
        <DialogActions>
          <Button onClick={() => setShowSessionDialog(false)}>Cancel</Button>
          <Button onClick={handleNewSession} variant="contained">Start New Session</Button>
        </DialogActions>
      </Dialog>
    </Container>
  );
};

export default ChatbotPage;