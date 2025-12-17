import React, { useState } from 'react';
import { Box, TextField, Button, IconButton } from '@mui/material';
import SendIcon from '@mui/icons-material/Send';

interface QueryInputProps {
  onSend: (query: string) => void;
  disabled?: boolean;
}

const QueryInput: React.FC<QueryInputProps> = ({ onSend, disabled = false }) => {
  const [query, setQuery] = useState('');

  const handleSubmit = (e: React.FormEvent) => {
    e.preventDefault();
    if (query.trim() && !disabled) {
      onSend(query.trim());
      setQuery('');
    }
  };

  const handleKeyPress = (e: React.KeyboardEvent) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSubmit(e);
    }
  };

  return (
    <Box
      component="form"
      onSubmit={handleSubmit}
      sx={{
        display: 'flex',
        alignItems: 'flex-end',
        gap: 1,
        p: 2,
        bgcolor: 'white',
        borderTop: '1px solid #e0e0e0'
      }}
    >
      <TextField
        fullWidth
        multiline
        maxRows={4}
        value={query}
        onChange={(e) => setQuery(e.target.value)}
        onKeyPress={handleKeyPress}
        placeholder="Ask about the textbook content..."
        variant="outlined"
        disabled={disabled}
        size="small"
      />
      <Button
        type="submit"
        variant="contained"
        disabled={!query.trim() || disabled}
        endIcon={<SendIcon />}
        sx={{ height: 'fit-content', mb: 1 }}
      >
        Send
      </Button>
    </Box>
  );
};

export default QueryInput;