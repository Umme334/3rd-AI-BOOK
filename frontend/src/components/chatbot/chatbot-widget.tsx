import React, { useState, useEffect, useRef } from 'react';
import axios from 'axios';
import './chatbot-widget.css';

import {
  ChatbotQueryRequest,
  ChatbotResponse,
  ChatMessage as ChatbotMessage,
  ChatbotSession
} from '../../types/chatbot';

interface ChatbotWidgetProps {
  textbookId: string;
  sessionId?: string;
  isOpen?: boolean;
}

const ChatbotWidget: React.FC<ChatbotWidgetProps> = ({ textbookId, sessionId, isOpen: isOpenProp }) => {
  const [isOpenState, setIsOpenState] = useState(false);
  const isOpen = isOpenProp !== undefined ? isOpenProp : isOpenState;

  const toggleOpen = () => {
    if (isOpenProp !== undefined) {
      // If isOpen is controlled by parent, we can't change it directly
      // In this case, we'll ignore the toggle if it's controlled
      // Or we could use a callback to notify parent
    } else {
      setIsOpenState(!isOpenState);
    }
  };
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

    } catch (error) {
      console.error('Error sending message:', error);
      const errorMessage: ChatbotMessage = {
        id: `msg-${Date.now()}-error`,
        role: 'assistant',
        content: 'Sorry, I encountered an error processing your request. Please try again.',
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
    <div className="chatbot-widget">
      {isOpen ? (
        <div className="chatbot-window">
          <div className="chatbot-header">
            <h3>Physical AI Assistant</h3>
            <button onClick={() => isOpenProp === undefined && setIsOpenState(false)}>×</button>
          </div>
          <div className="chatbot-messages">
            {messages.length === 0 ? (
              <div className="welcome-message">
                <p>Hello! I'm your Physical AI & Humanoid Robotics assistant.</p>
                <p>Ask me questions about the textbook content, and I'll provide answers based on the material.</p>
                <p><em>Select text in the textbook and ask questions about it!</em></p>
              </div>
            ) : (
              messages.map((message) => (
                <div
                  key={message.id}
                  className={`message ${message.role}`}
                >
                  <div className="message-content">{message.content}</div>
                  <div className="message-timestamp">
                    {new Date(message.timestamp).toLocaleTimeString()}
                  </div>
                </div>
              ))
            )}
            {isLoading && (
              <div className="message bot">
                <div className="message-content">Thinking...</div>
              </div>
            )}
            <div ref={messagesEndRef} />
          </div>
          {selectedText && (
            <div className="selected-text-preview">
              <small><strong>Selected:</strong> {selectedText.substring(0, 60)}{selectedText.length > 60 ? '...' : ''}</small>
              <button
                className="clear-selection"
                onClick={() => setSelectedText('')}
                type="button"
              >
                Clear
              </button>
            </div>
          )}
          <form onSubmit={handleSubmit} className="chatbot-input-form">
            <input
              type="text"
              value={inputMessage}
              onChange={(e) => setInputMessage(e.target.value)}
              placeholder={selectedText
                ? "Ask a question about the selected text..."
                : "Ask a question about the textbook..."}
              disabled={isLoading}
            />
            <button type="submit" disabled={isLoading}>
              Send
            </button>
          </form>
        </div>
      ) : (
        <button className="chatbot-toggle" onClick={() => isOpenProp === undefined && setIsOpenState(true)}>
          🤖 AI Assistant
        </button>
      )}
    </div>
  );
};

export default ChatbotWidget;
export { ChatbotWidget };