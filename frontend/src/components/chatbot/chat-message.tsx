import React, { useState, useEffect, useRef } from 'react';
import { Box, TextField, Button, Paper, Typography, Divider, Avatar } from '@mui/material';
import SendIcon from '@mui/icons-material/Send';

interface ChatMessageData {
  id: string;
  role: 'user' | 'assistant';
  content: string;
  timestamp: Date;
}

interface ChatMessageProps {
  message: ChatMessageData;
  isTyping?: boolean;
}

export const ChatMessage: React.FC<ChatMessageProps> = ({ message, isTyping = false }) => {
  const isUser = message.role === 'user';

  return (
    <Box sx={{
      display: 'flex',
      justifyContent: isUser ? 'flex-end' : 'flex-start',
      mb: 2
    }}>
      {!isUser && (
        <Avatar
          sx={{
            bgcolor: 'primary.main',
            mr: 1,
            width: 32,
            height: 32
          }}
        >
          AI
        </Avatar>
      )}
      <Paper
        sx={{
          p: 1.5,
          ml: isUser ? 2 : 0,
          mr: isUser ? 0 : 2,
          bgcolor: isUser ? 'primary.main' : 'grey.100',
          color: isUser ? 'white' : 'text.primary',
          maxWidth: '70%',
          wordWrap: 'break-word'
        }}
      >
        <Typography variant="body2">{message.content}</Typography>
        {isTyping && (
          <Box sx={{ display: 'flex', gap: 0.5, mt: 0.5 }}>
            <Box sx={{ width: 6, height: 6, bgcolor: 'grey.500', borderRadius: '50%', animation: 'pulse 1.5s infinite' }} />
            <Box sx={{ width: 6, height: 6, bgcolor: 'grey.500', borderRadius: '50%', animation: 'pulse 1.5s infinite', animationDelay: '0.5s' }} />
            <Box sx={{ width: 6, height: 6, bgcolor: 'grey.500', borderRadius: '50%', animation: 'pulse 1.5s infinite', animationDelay: '1s' }} />
          </Box>
        )}
        <Typography variant="caption" sx={{ mt: 0.5, display: 'block', opacity: 0.7 }}>
          {message.timestamp.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' })}
        </Typography>
      </Paper>
      {isUser && (
        <Avatar
          sx={{
            bgcolor: 'secondary.main',
            ml: 1,
            width: 32,
            height: 32
          }}
        >
          U
        </Avatar>
      )}
    </Box>
  );
};

interface ChatbotWidgetProps {
  textbookId: string;
  sessionId?: string;
}

export const ChatbotWidget: React.FC<ChatbotWidgetProps> = ({ textbookId, sessionId }) => {
  const [messages, setMessages] = useState<ChatMessageData[]>([
    {
      id: '1',
      role: 'assistant',
      content: 'Hello! I\'m your AI assistant for Physical AI and Humanoid Robotics. How can I help you with the textbook content?',
      timestamp: new Date()
    }
  ]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const messagesEndRef = useRef<HTMLDivElement>(null);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const handleSendMessage = async () => {
    if (!inputValue.trim() || isLoading) return;

    // Add user message
    const userMessage: ChatMessageData = {
      id: Date.now().toString(),
      role: 'user',
      content: inputValue,
      timestamp: new Date()
    };

    setMessages(prev => [...prev, userMessage]);
    setInputValue('');
    setIsLoading(true);

    try {
      // Simulate API call to backend
      await new Promise(resolve => setTimeout(resolve, 1000)); // Simulate network delay

      // In a real implementation, this would call the backend API
      // const response = await fetch('/api/chatbot/query', {
      //   method: 'POST',
      //   headers: { 'Content-Type': 'application/json' },
      //   body: JSON.stringify({
      //     textbook_id: textbookId,
      //     session_id: sessionId,
      //     query: inputValue
      //   })
      // });

      // const result = await response.json();

      // Simulate AI response
      const aiResponse: ChatMessageData = {
        id: (Date.now() + 1).toString(),
        role: 'assistant',
        content: `I understand you're asking about: "${inputValue}". This is a simulated response from the RAG chatbot. In a real implementation, I would search the textbook content for relevant information about Physical AI and Humanoid Robotics to answer your question.`,
        timestamp: new Date()
      };

      setMessages(prev => [...prev, aiResponse]);
    } catch (error) {
      const errorMessage: ChatMessageData = {
        id: (Date.now() + 1).toString(),
        role: 'assistant',
        content: 'Sorry, I encountered an error processing your request. Please try again.',
        timestamp: new Date()
      };
      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  const handleKeyPress = (e: React.KeyboardEvent) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSendMessage();
    }
  };

  return (
    <Box sx={{
      display: 'flex',
      flexDirection: 'column',
      height: '600px',
      border: '1px solid #e0e0e0',
      borderRadius: 2,
      overflow: 'hidden'
    }}>
      <Paper
        elevation={0}
        sx={{
          p: 2,
          bgcolor: 'primary.main',
          color: 'white',
          display: 'flex',
          alignItems: 'center'
        }}
      >
        <Typography variant="h6" component="div">
          Physical AI & Humanoid Robotics Assistant
        </Typography>
      </Paper>

      <Box
        sx={{
          flex: 1,
          overflowY: 'auto',
          p: 2,
          bgcolor: 'grey.50'
        }}
      >
        {messages.map((message) => (
          <ChatMessage
            key={message.id}
            message={message}
          />
        ))}
        {isLoading && (
          <ChatMessage
            message={{
              id: 'loading',
              role: 'assistant',
              content: 'Thinking...',
              timestamp: new Date()
            }}
            isTyping={true}
          />
        )}
        <div ref={messagesEndRef} />
      </Box>

      <Divider />

      <Box sx={{ p: 2, bgcolor: 'white' }}>
        <Box sx={{ display: 'flex', gap: 1 }}>
          <TextField
            fullWidth
            multiline
            maxRows={4}
            value={inputValue}
            onChange={(e) => setInputValue(e.target.value)}
            onKeyPress={handleKeyPress}
            placeholder="Ask about Physical AI, ROS 2, Gazebo, NVIDIA Isaac, Vision-Language-Action..."
            variant="outlined"
            disabled={isLoading}
            size="small"
          />
          <Button
            variant="contained"
            onClick={handleSendMessage}
            disabled={!inputValue.trim() || isLoading}
            endIcon={<SendIcon />}
            sx={{ height: 'fit-content' }}
          >
            Send
          </Button>
        </Box>
        <Typography variant="caption" color="text.secondary" sx={{ mt: 1, display: 'block' }}>
          Ask questions about the textbook content. The AI will respond based on the Physical AI and Humanoid Robotics material.
        </Typography>
      </Box>
    </Box>
  );
};