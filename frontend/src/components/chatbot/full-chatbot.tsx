import React, { useState, useEffect, useRef } from 'react';
import axios, { AxiosError } from 'axios';
import {
  ChatbotQueryRequest,
  ChatbotResponse,
  ChatMessage as ChatbotMessage,
} from '../../types/chatbot';

interface FullChatbotProps {
  textbookId: string;
  sessionId?: string;
}

const FullChatbot: React.FC<FullChatbotProps> = ({ textbookId, sessionId }) => {
  const [messages, setMessages] = useState<ChatbotMessage[]>([]);
  const [inputMessage, setInputMessage] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [currentSessionId, setCurrentSessionId] = useState(sessionId || '');
  const [selectedText, setSelectedText] = useState<string>('');
  const messagesEndRef = useRef<null | HTMLDivElement>(null);

  // Function to send a message to the backend API
  const sendMessage = async (query: string, selectedText?: string) => {
    if (!query.trim()) return;

    setIsLoading(true);

    try {
      // Add user message to the chat
      const userMessage: ChatbotMessage = {
        id: `msg-${Date.now()}-user`,
        role: 'user',
        content: query,
        timestamp: new Date().toISOString()
      };

      setMessages(prev => [...prev, userMessage]);

      // Prepare the query request with selected text if available
      const request: ChatbotQueryRequest = {
        query,
        textbook_id: textbookId,
        session_id: currentSessionId,
        selected_text: selectedText || undefined
      };

      // Call the backend API
      const response = await axios.post<ChatbotResponse>(
        `${import.meta.env.VITE_API_BASE_URL || 'http://localhost:8000'}/chatbot/query`,
        request,
        {
          headers: {
            'Content-Type': 'application/json',
          }
        }
      );

      const botResponse: ChatbotMessage = {
        id: `msg-${Date.now()}-bot`,
        role: 'assistant',
        content: response.data.response,
        timestamp: new Date().toISOString(),
        sources: response.data.context_sources
      };

      setMessages(prev => [...prev, botResponse]);
      setIsLoading(false);
      setInputMessage('');
      // Clear selected text after sending
      setSelectedText('');

    } catch (error: unknown) {
      console.error('Error sending message:', error);
      let errorMessageContent = 'Sorry, I encountered an error processing your request. Please try again.';

      // Check if it's an axios error with response
      if (axios.isAxiosError(error) && error.response) {
        if (error.response.status === 404 && error.response.data?.detail?.includes('Textbook with ID')) {
          errorMessageContent = 'This textbook does not exist yet. The AI assistant needs an existing textbook to answer questions. Try creating a textbook first or use a different textbook ID.';
        } else {
          errorMessageContent = error.response.data?.detail || error.response.statusText || errorMessageContent;
        }
      } else if (error instanceof Error) {
        errorMessageContent = error.message;
      }

      const errorMessage: ChatbotMessage = {
        id: `msg-${Date.now()}-error`,
        role: 'assistant',
        content: errorMessageContent,
        timestamp: new Date().toISOString()
      };
      setMessages(prev => [...prev, errorMessage]);
      setIsLoading(false);
    }
  };

  const handleSubmit = (e: React.FormEvent) => {
    e.preventDefault();
    sendMessage(inputMessage, selectedText);
  };

  // Function to handle text selection
  const handleTextSelection = () => {
    const selectedText = window.getSelection()?.toString();
    if (selectedText && selectedText.trim().length > 0) {
      setSelectedText(selectedText);
    }
  };

  // Add event listener for text selection
  useEffect(() => {
    const handleMouseUp = () => {
      handleTextSelection();
    };

    document.addEventListener('mouseup', handleMouseUp);
    return () => {
      document.removeEventListener('mouseup', handleMouseUp);
    };
  }, []);

  // Scroll to bottom of messages
  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [messages]);

  return (
    <div style={{ display: 'flex', flexDirection: 'column', height: '100%', width: '100%' }}>
      <div style={{
        flex: 1,
        padding: '20px',
        overflowY: 'auto',
        backgroundColor: '#f9f9f9',
        borderRadius: '8px',
        marginBottom: '15px'
      }}>
        {messages.length === 0 ? (
          <div style={{ textAlign: 'center', padding: '40px 0', color: '#666' }}>
            <h3>Hello! I'm your Physical AI & Humanoid Robotics assistant.</h3>
            <p>Ask me questions about the textbook content, and I'll provide answers based on the material.</p>
            <p><em>Select text in the textbook and ask questions about it!</em></p>
          </div>
        ) : (
          <div>
            {messages.map((message) => (
              <div
                key={message.id}
                style={{
                  maxWidth: '80%',
                  padding: '12px 16px',
                  margin: '10px 0',
                  borderRadius: '18px',
                  wordWrap: 'break-word',
                  backgroundColor: message.role === 'user' ? '#e3f2fd' : '#f5f5f5',
                  alignSelf: message.role === 'user' ? 'flex-end' : 'flex-start',
                  marginLeft: message.role === 'user' ? 'auto' : '0',
                  marginRight: message.role === 'user' ? '0' : 'auto',
                }}
              >
                <div style={{ marginBottom: '5px' }}>{message.content}</div>
                <div style={{ fontSize: '12px', color: '#999', textAlign: 'right' }}>
                  {new Date(message.timestamp).toLocaleTimeString()}
                </div>
              </div>
            ))}
            {isLoading && (
              <div style={{
                maxWidth: '80%',
                padding: '12px 16px',
                margin: '10px 0',
                borderRadius: '18px',
                backgroundColor: '#f5f5f5',
                alignSelf: 'flex-start'
              }}>
                <div>Thinking...</div>
              </div>
            )}
            <div ref={messagesEndRef} />
          </div>
        )}
      </div>

      {selectedText && (
        <div style={{
          display: 'flex',
          justifyContent: 'space-between',
          alignItems: 'center',
          padding: '10px',
          backgroundColor: '#e8f5e9',
          border: '1px solid #c8e6c9',
          borderRadius: '4px',
          marginBottom: '10px'
        }}>
          <small style={{ color: '#2e7d32' }}>
            <strong>Selected:</strong> {selectedText.substring(0, 60)}{selectedText.length > 60 ? '...' : ''}
          </small>
          <button
            style={{
              background: 'none',
              border: '1px solid #4caf50',
              color: '#4caf50',
              padding: '4px 8px',
              borderRadius: '4px',
              cursor: 'pointer',
              fontSize: '12px'
            }}
            onClick={() => setSelectedText('')}
            type="button"
          >
            Clear
          </button>
        </div>
      )}

      <form onSubmit={handleSubmit} style={{ display: 'flex', gap: '10px' }}>
        <input
          type="text"
          value={inputMessage}
          onChange={(e) => setInputMessage(e.target.value)}
          placeholder={selectedText
            ? "Ask a question about the selected text..."
            : "Ask a question about the textbook..."}
          disabled={isLoading}
          style={{
            flex: 1,
            padding: '12px',
            border: '1px solid #ddd',
            borderRadius: '24px',
            fontSize: '14px'
          }}
        />
        <button
          type="submit"
          disabled={isLoading}
          style={{
            backgroundColor: '#1976d2',
            color: 'white',
            border: 'none',
            borderRadius: '24px',
            padding: '12px 20px',
            cursor: 'pointer'
          }}
        >
          Send
        </button>
      </form>
    </div>
  );
};

export default FullChatbot;