import React, { useState, useEffect, useRef } from 'react';
import Layout from '@theme/Layout';

interface ChatbotMessage {
  id: string;
  role: 'user' | 'assistant';
  content: string;
  timestamp: string;
  sources?: string[];
}

interface ChatbotQueryRequest {
  query: string;
  textbook_id: string;
  session_id?: string;
  selected_text?: string;
}

interface ChatbotResponse {
  response: string;
  session_id: string;
  sources?: any[];
  context_sources?: string[];
  followup_questions?: string[];
  query_time?: string;
  confidence?: number;
  tokens_used?: number;
}

interface ChatbotWidgetProps {
  textbookId: string;
  sessionId?: string;
}

const ChatbotWidget: React.FC<ChatbotWidgetProps> = ({ textbookId, sessionId }) => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState<ChatbotMessage[]>([]);
  const [inputMessage, setInputMessage] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [currentSessionId, setCurrentSessionId] = useState(sessionId || '');
  const [selectedText, setSelectedText] = useState<string>('');
  const [isAuthenticated, setIsAuthenticated] = useState(true); // Default to true to show the chatbot
  const [loginForm, setLoginForm] = useState({ email: '', password: '' });
  const [loginLoading, setLoginLoading] = useState(false);
  const [loginError, setLoginError] = useState('');
  const messagesEndRef = useRef<null | HTMLDivElement>(null);

  // Check authentication status on component mount
  useEffect(() => {
    const checkAuth = async () => {
      try {
        // Check if user has a valid session by calling the session endpoint
        const response = await fetch('http://localhost:8000/auth/better-auth/session', {
          headers: {
            'Authorization': `Bearer ${localStorage.getItem('better-auth-session') || ''}`
          }
        });

        if (response.ok) {
          const data = await response.json();
          setIsAuthenticated(!!data.valid);
        } else {
          setIsAuthenticated(false);
        }
      } catch (error) {
        // If there's an error checking auth, assume user is not authenticated
        setIsAuthenticated(false);
      }
    };

    checkAuth();
  }, []);

  // Handle login
  const handleLogin = async (e: React.FormEvent) => {
    e.preventDefault();
    setLoginLoading(true);
    setLoginError('');

    try {
      // Call the backend login API
      const response = await fetch('http://localhost:8000/auth/signin', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          email: loginForm.email,
          password: loginForm.password
        })
      });

      if (response.ok) {
        const data = await response.json();
        // Store the session token
        localStorage.setItem('better-auth-session', data.session_token || '');
        // Check auth status again to update the UI
        setIsAuthenticated(true);
        // Clear the login form
        setLoginForm({ email: '', password: '' });
      } else {
        const errorData = await response.json();
        setLoginError(errorData.message || 'Invalid email or password');
      }
    } catch (error) {
      console.error('Login error:', error);
      setLoginError('An error occurred during login. Please try again.');
    } finally {
      setLoginLoading(false);
    }
  };

  // Handle showing signup (redirect to signup page)
  const handleShowSignup = () => {
    window.location.href = '/signup';
  };

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
        textbook_id: textbookId || 'physical-ai-textbook',
        session_id: currentSessionId,
        selected_text: selectedText || undefined
      };

      // Call the backend API
      const response = await fetch('http://localhost:8000/chatbot/query', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(request),
      });

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      const data: ChatbotResponse = await response.json();

      const botResponse: ChatbotMessage = {
        id: `msg-${Date.now()}-bot`,
        role: 'assistant',
        content: data.response,
        timestamp: new Date().toISOString(),
        sources: data.context_sources
      };

      setMessages(prev => [...prev, botResponse]);
      setIsLoading(false);
      setInputMessage('');
      // Clear selected text after sending
      setSelectedText('');

      // Update session ID if it was returned
      if (data.session_id && !currentSessionId) {
        setCurrentSessionId(data.session_id);
      }

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

  if (!isAuthenticated) {
    return (
      <div style={{
        position: 'fixed',
        bottom: '20px',
        right: '20px',
        zIndex: 1000
      }}>
        <div style={{
          backgroundColor: '#fff3cd',
          border: '1px solid #ffeaa7',
          borderRadius: '8px',
          padding: '12px 16px',
          minWidth: '300px',
          boxShadow: '0 4px 12px rgba(0, 0, 0, 0.1)'
        }}>
          <div style={{ marginBottom: '10px' }}>
            <div style={{ fontWeight: 'bold', color: '#856404' }}>Authentication Required</div>
            <div style={{ fontSize: '12px', color: '#856404', marginTop: '4px' }}>
              Please sign in to access the Physical AI Assistant
            </div>
          </div>

          {/* Simple inline login form */}
          <div style={{ display: 'flex', flexDirection: 'column', gap: '8px' }}>
            <input
              type="email"
              placeholder="Email"
              value={loginForm.email}
              onChange={(e) => setLoginForm({...loginForm, email: e.target.value})}
              style={{
                padding: '8px',
                borderRadius: '4px',
                border: '1px solid #ddd'
              }}
            />
            <input
              type="password"
              placeholder="Password"
              value={loginForm.password}
              onChange={(e) => setLoginForm({...loginForm, password: e.target.value})}
              style={{
                padding: '8px',
                borderRadius: '4px',
                border: '1px solid #ddd'
              }}
            />
            <div style={{ display: 'flex', gap: '8px' }}>
              <button
                onClick={handleLogin}
                disabled={loginLoading}
                style={{
                  flex: 1,
                  backgroundColor: loginLoading ? '#cccccc' : '#1976d2',
                  color: 'white',
                  border: 'none',
                  borderRadius: '4px',
                  padding: '8px',
                  cursor: loginLoading ? 'not-allowed' : 'pointer'
                }}
              >
                {loginLoading ? 'Signing in...' : 'Sign In'}
              </button>
              <button
                onClick={handleShowSignup}
                style={{
                  flex: 1,
                  backgroundColor: '#4caf50',
                  color: 'white',
                  border: 'none',
                  borderRadius: '4px',
                  padding: '8px',
                  cursor: 'pointer'
                }}
              >
                Sign Up
              </button>
            </div>
            {loginError && (
              <div style={{ color: '#dc3545', fontSize: '12px' }}>
                {loginError}
              </div>
            )}
          </div>
        </div>
      </div>
    );
  }

  return (
    <div className="chatbot-widget">
      {isOpen ? (
        <div className="chatbot-window">
          <div className="chatbot-header" style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center' }}>
            <div>
              <h3 style={{ margin: 0, fontSize: '16px' }}>Physical AI Assistant</h3>
              <div style={{ fontSize: '12px', opacity: 0.9, marginTop: '2px' }}>
                Ask about: {textbookId}
              </div>
            </div>
            <div style={{ display: 'flex', alignItems: 'center', gap: '8px' }}>
              <button
                onClick={() => {
                  // Clear the local session and redirect to sign out
                  localStorage.removeItem('better-auth-session');
                  window.location.href = '/login';
                }}
                style={{
                  background: 'rgba(255, 255, 255, 0.2)',
                  border: 'none',
                  color: 'white',
                  fontSize: '12px',
                  cursor: 'pointer',
                  padding: '6px 10px',
                  borderRadius: '4px',
                  display: 'flex',
                  alignItems: 'center'
                }}
                title="Sign Out"
              >
                Sign Out
              </button>
              <button
                onClick={() => setIsOpen(false)}
                style={{
                  background: 'none',
                  border: 'none',
                  color: 'white',
                  fontSize: '20px',
                  cursor: 'pointer',
                  padding: '0',
                  width: '30px',
                  height: '30px',
                  borderRadius: '50%',
                  display: 'flex',
                  alignItems: 'center',
                  justifyContent: 'center'
                }}
              >
                ×
              </button>
            </div>
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
        <button className="chatbot-toggle" onClick={() => setIsOpen(true)}>
          🤖 AI Assistant
        </button>
      )}
    </div>
  );
};

export default ChatbotWidget;