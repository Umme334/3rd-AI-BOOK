import React, { useState, useEffect, useRef } from 'react';
import BrowserOnly from '@docusaurus/BrowserOnly';

interface ChatMessage {
  id: string;
  role: 'user' | 'assistant';
  content: string;
  timestamp: string;
}

interface ContextualChatbotProps {
  textbookId: string;
  chapterId?: string;
  sectionId?: string;
  context?: string; // Additional context from the current page
  sessionId?: string;
  title?: string; // Custom title for the chatbot
}

const ContextualChatbot: React.FC<ContextualChatbotProps> = ({
  textbookId,
  chapterId,
  sectionId,
  context,
  sessionId,
  title = 'Physical AI Assistant'
}) => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState<ChatMessage[]>([]);
  const [inputMessage, setInputMessage] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [currentSessionId, setCurrentSessionId] = useState(sessionId || '');
  const messagesEndRef = useRef<null | HTMLDivElement>(null);

  // Function to send a message to the backend API
  const sendMessage = async (query: string) => {
    if (!query.trim()) return;

    setIsLoading(true);

    try {
      // Add user message to the chat
      const userMessage: ChatMessage = {
        id: `msg-${Date.now()}-user`,
        role: 'user',
        content: query,
        timestamp: new Date().toISOString()
      };

      setMessages(prev => [...prev, userMessage]);

      // Prepare the query request with additional context
      const request = {
        query,
        textbook_id: textbookId,
        session_id: currentSessionId,
        context: {
          chapter_id: chapterId,
          section_id: sectionId,
          page_context: context,
          source_page: typeof window !== 'undefined' ? window.location.pathname : ''
        }
      };

      // Call the backend API
      const response = await fetch('/api/chatbot/query', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(request),
      });

      if (!response.ok) {
        throw new Error(`Failed to get response: ${response.statusText}`);
      }

      const data = await response.json();

      const botResponse: ChatMessage = {
        id: `msg-${Date.now()}-bot`,
        role: 'assistant',
        content: data.response,
        timestamp: new Date().toISOString()
      };

      setMessages(prev => [...prev, botResponse]);
      setIsLoading(false);
      setInputMessage('');

      // Update session ID if it was created
      if (data.session_id && !currentSessionId) {
        setCurrentSessionId(data.session_id);
      }
    } catch (error) {
      console.error('Error sending message:', error);

      const errorMessage: ChatMessage = {
        id: `msg-${Date.now()}-error`,
        role: 'assistant',
        content: 'Sorry, I encountered an error. Please try again.',
        timestamp: new Date().toISOString()
      };

      setMessages(prev => [...prev, errorMessage]);
      setIsLoading(false);
      setInputMessage('');
    }
  };

  const handleSubmit = (e: React.FormEvent) => {
    e.preventDefault();
    sendMessage(inputMessage);
  };

  // Scroll to bottom of messages
  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [messages]);

  // Toggle chatbot visibility
  const toggleChatbot = () => {
    setIsOpen(!isOpen);
  };

  // Close chatbot
  const closeChatbot = () => {
    setIsOpen(false);
  };

  return (
    <div className="contextual-chatbot">
      {isOpen ? (
        <div className="chatbot-window contextual">
          <div className="chatbot-header">
            <h3>{title}</h3>
            <div className="chatbot-context-info">
              {chapterId && <span className="chapter-tag">Chapter: {chapterId}</span>}
              {sectionId && <span className="section-tag">Section: {sectionId}</span>}
            </div>
            <button onClick={closeChatbot} className="chatbot-close-btn">×</button>
          </div>
          <div className="chatbot-messages">
            {messages.length === 0 ? (
              <div className="welcome-message">
                <p>Hello! I'm your {title}.</p>
                <p>Ask me questions about this content, and I'll provide answers based on the textbook material.</p>
                {context && (
                  <p className="context-hint">
                    I have context about the current section to provide more relevant answers.
                  </p>
                )}
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
          <form onSubmit={handleSubmit} className="chatbot-input-form">
            <input
              type="text"
              value={inputMessage}
              onChange={(e) => setInputMessage(e.target.value)}
              placeholder="Ask a question about this content..."
              disabled={isLoading}
              aria-label="Chat message input"
            />
            <button type="submit" disabled={isLoading} aria-label="Send message">
              Send
            </button>
          </form>
        </div>
      ) : (
        <button
          className="chatbot-toggle contextual"
          onClick={toggleChatbot}
          aria-label="Open AI Assistant"
        >
          🤖 {title.split(' ')[0]} Chat
        </button>
      )}
    </div>
  );
};

// Wrapper to ensure component only runs in browser environment
const ContextualChatbotWrapper: React.FC<ContextualChatbotProps> = (props) => {
  return (
    <BrowserOnly fallback={<div>Loading contextual chatbot...</div>}>
      {() => <ContextualChatbot {...props} />}
    </BrowserOnly>
  );
};

export default ContextualChatbotWrapper;