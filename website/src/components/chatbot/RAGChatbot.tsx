import React, { useState, useEffect, useRef } from 'react';
import { useLocation } from '@docusaurus/router';

interface RAGChatbotProps {
  textbookId?: string;
  chapterId?: string;
  sectionTitle?: string;
}

interface Message {
  id: string;
  role: 'user' | 'assistant';
  content: string;
  timestamp: string;
  sources?: any[];
}

const RAGChatbot: React.FC<RAGChatbotProps> = ({
  textbookId = 'physical-ai-textbook',
  chapterId,
  sectionTitle
}) => {
  const [selectedText, setSelectedText] = useState('');
  const [isWidgetVisible, setIsWidgetVisible] = useState(false);
  const [messages, setMessages] = useState<Message[]>([]);
  const [inputMessage, setInputMessage] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [isAuthenticated, setIsAuthenticated] = useState(false);
  const location = useLocation();
  const messagesEndRef = useRef<HTMLDivElement>(null);

  // Check authentication status on component mount
  useEffect(() => {
    // In a real implementation, you would check the user's auth status
    // For now, we'll simulate checking localStorage or auth state
    const checkAuth = () => {
      // Simulate checking if user is authenticated
      const token = localStorage.getItem('better-auth-session');
      setIsAuthenticated(!!token);
    };

    checkAuth();
  }, []);

  // Handle text selection
  useEffect(() => {
    const handleSelection = () => {
      const selectedTextObj = window.getSelection();
      if (selectedTextObj) {
        const text = selectedTextObj.toString().trim();
        if (text && text.length > 0 && text.length < 500) { // Limit selection length
          setSelectedText(text);
        }
      }
    };

    document.addEventListener('mouseup', handleSelection);
    return () => {
      document.removeEventListener('mouseup', handleSelection);
    };
  }, []);

  // Scroll to bottom of messages
  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [messages]);

  const toggleWidget = () => {
    setIsWidgetVisible(!isWidgetVisible);
  };

  const handleSendMessage = async () => {
    if (!inputMessage.trim() || isLoading) return;

    const userMessage: Message = {
      id: `msg-${Date.now()}`,
      role: 'user',
      content: inputMessage,
      timestamp: new Date().toISOString()
    };

    setMessages(prev => [...prev, userMessage]);
    setIsLoading(true);
    const currentInput = inputMessage;
    const currentSelectedText = selectedText;

    // Clear input and selected text
    setInputMessage('');
    setSelectedText('');

    try {
      // Prepare the query data with context
      const queryData = {
        query: currentInput,
        textbook_id: textbookId,
        session_id: `session-${Date.now()}-${Math.random().toString(36).substr(2, 9)}`,
        selected_text: currentSelectedText || undefined,
        context: {
          chapter_id: chapterId,
          section_title: sectionTitle || document.title,
          current_url: location.pathname,
          current_page_content: currentSelectedText || currentInput
        }
      };

      // Call the backend RAG API
      const response = await fetch('http://localhost:8000/chatbot/query', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(queryData)
      });

      if (!response.ok) {
        throw new Error(`API call failed: ${response.status} ${response.statusText}`);
      }

      const data = await response.json();

      // Add bot response to chat
      const botMessage: Message = {
        id: `msg-${Date.now() + 1}`,
        role: 'assistant',
        content: data.response,
        timestamp: new Date().toISOString(),
        sources: data.sources || data.context_sources
      };

      setMessages(prev => [...prev, botMessage]);
    } catch (error) {
      console.error('Error sending message:', error);

      const errorMessage: Message = {
        id: `msg-${Date.now() + 1}`,
        role: 'assistant',
        content: 'Sorry, I encountered an error processing your request. Please try again.',
        timestamp: new Date().toISOString()
      };

      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  const handleKeyDown = (e: React.KeyboardEvent<HTMLInputElement>) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSendMessage();
    }
  };

  const handleSignOut = async () => {
    try {
      // Call the backend signout endpoint
      const response = await fetch('http://localhost:8000/auth/signout', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          'Authorization': `Bearer ${localStorage.getItem('better-auth-session') || ''}`
        }
      });

      if (response.ok) {
        // Clear the local session
        localStorage.removeItem('better-auth-session');

        // Update authentication state
        setIsAuthenticated(false);
      } else {
        const data = await response.json();
        console.error('Signout error:', data.detail || 'Failed to sign out');
      }
    } catch (error) {
      console.error('Error signing out:', error);
    }
  };

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
          minWidth: '250px',
          boxShadow: '0 4px 12px rgba(0, 0, 0, 0.1)'
        }}>
          <div style={{ display: 'flex', alignItems: 'center', justifyContent: 'space-between' }}>
            <div>
              <div style={{ fontWeight: 'bold', color: '#856404' }}>Authentication Required</div>
              <div style={{ fontSize: '12px', color: '#856404', marginTop: '4px' }}>
                Please sign in to access the Physical AI Assistant
              </div>
            </div>
            <button
              onClick={() => window.location.href = '/login'}
              style={{
                backgroundColor: '#1976d2',
                color: 'white',
                border: 'none',
                borderRadius: '4px',
                padding: '6px 12px',
                cursor: 'pointer',
                fontSize: '12px'
              }}
            >
              Sign In
            </button>
          </div>
        </div>
      </div>
    );
  }

  return (
    <div style={{
      position: 'fixed',
      bottom: '20px',
      right: '20px',
      zIndex: 1000
    }}>
      {!isWidgetVisible ? (
        <button
          onClick={toggleWidget}
          style={{
            backgroundColor: '#1976d2',
            color: 'white',
            border: 'none',
            borderRadius: '50%',
            width: '60px',
            height: '60px',
            fontSize: '24px',
            cursor: 'pointer',
            boxShadow: '0 4px 12px rgba(0, 0, 0, 0.15)',
            display: 'flex',
            alignItems: 'center',
            justifyContent: 'center'
          }}
          title="Open Physical AI Assistant"
        >
          🤖
        </button>
      ) : (
        <div style={{
          width: '100%',
          height: '100%',
          backgroundColor: 'white',
          border: '1px solid #ddd',
          borderRadius: '12px',
          boxShadow: '0 8px 30px rgba(0, 0, 0, 0.12)',
          display: 'flex',
          flexDirection: 'column',
          overflow: 'hidden'
        }}>
          <div style={{
            backgroundColor: '#1976d2',
            color: 'white',
            padding: '16px',
            display: 'flex',
            justifyContent: 'space-between',
            alignItems: 'center'
          }}>
            <div>
              <h3 style={{ margin: 0, fontSize: '16px' }}>Physical AI Assistant</h3>
              <div style={{ fontSize: '12px', opacity: 0.9, marginTop: '2px' }}>
                Ask about: {sectionTitle || 'Current Page'}
              </div>
            </div>
            <div style={{ display: 'flex', alignItems: 'center', gap: '8px' }}>
              <button
                onClick={handleSignOut}
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
                onClick={toggleWidget}
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

          <div style={{
            flex: 1,
            padding: '16px',
            overflowY: 'auto',
            display: 'flex',
            flexDirection: 'column',
            gap: '12px',
            backgroundColor: '#fafafa'
          }}>
            {messages.length === 0 ? (
              <div style={{
                textAlign: 'center',
                color: '#666',
                marginTop: '20px',
                display: 'flex',
                flexDirection: 'column',
                alignItems: 'center'
              }}>
                <div style={{ fontSize: '48px', marginBottom: '16px' }}>🤖</div>
                <h4 style={{ margin: '0 0 8px 0', color: '#1976d2' }}>Physical AI Assistant</h4>
                <p style={{ margin: '0 0 16px 0', maxWidth: '300px' }}>
                  Ask me questions about the Physical AI and Humanoid Robotics content you're reading.
                </p>
                {selectedText && (
                  <div style={{
                    backgroundColor: '#e3f2fd',
                    padding: '12px',
                    borderRadius: '8px',
                    fontSize: '13px',
                    maxWidth: '300px',
                    border: '1px solid #bbdefb',
                    marginTop: '8px'
                  }}>
                    <strong style={{ display: 'block', marginBottom: '4px' }}>Selected text:</strong>
                    "{selectedText.substring(0, 80)}{selectedText.length > 80 ? '...' : ''}"
                  </div>
                )}
              </div>
            ) : (
              messages.map((message) => (
                <div
                  key={message.id}
                  style={{
                    maxWidth: '85%',
                    padding: '12px 16px',
                    borderRadius: message.role === 'user'
                      ? '18px 4px 18px 18px'
                      : '4px 18px 18px 18px',
                    backgroundColor: message.role === 'user' ? '#1976d2' : '#e9ecef',
                    color: message.role === 'user' ? 'white' : '#333',
                    alignSelf: message.role === 'user' ? 'flex-end' : 'flex-start',
                    fontSize: '14px',
                    lineHeight: '1.4'
                  }}
                >
                  <div>{message.content}</div>
                  {message.sources && message.sources.length > 0 && (
                    <div style={{
                      fontSize: '11px',
                      marginTop: '6px',
                      paddingTop: '6px',
                      borderTop: '1px solid rgba(0,0,0,0.1)'
                    }}>
                      <strong>Sources:</strong> {message.sources.map((src: any, idx: number) =>
                        <span key={idx}>{src.title || src.section_title || src.chapter_title}{idx < message.sources.length - 1 ? ', ' : ''}</span>
                      )}
                    </div>
                  )}
                </div>
              ))
            )}
            {isLoading && (
              <div style={{
                maxWidth: '85%',
                padding: '12px 16px',
                borderRadius: '4px 18px 18px 18px',
                backgroundColor: '#e9ecef',
                color: '#333',
                alignSelf: 'flex-start',
                fontStyle: 'italic'
              }}>
                Thinking...
              </div>
            )}
            <div ref={messagesEndRef} />
          </div>

          {selectedText && (
            <div style={{
              padding: '8px 12px',
              backgroundColor: '#e3f2fd',
              border: '1px solid #bbdefb',
              borderTop: 'none',
              fontSize: '12px'
            }}>
              <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center' }}>
                <span>
                  <strong>Selected:</strong> {selectedText.substring(0, 60)}{selectedText.length > 60 ? '...' : ''}
                </span>
                <button
                  onClick={() => setSelectedText('')}
                  style={{
                    background: '#1976d2',
                    color: 'white',
                    border: 'none',
                    borderRadius: '4px',
                    padding: '2px 8px',
                    fontSize: '10px',
                    cursor: 'pointer'
                  }}
                >
                  Clear
                </button>
              </div>
            </div>
          )}

          <div style={{
            padding: '12px',
            backgroundColor: 'white',
            borderTop: '1px solid #ddd',
            display: 'flex'
          }}>
            <input
              type="text"
              value={inputMessage}
              onChange={(e) => setInputMessage(e.target.value)}
              onKeyDown={handleKeyDown}
              placeholder={selectedText
                ? "Ask about selected text..."
                : "Ask about this content..."}
              disabled={isLoading}
              style={{
                flex: 1,
                padding: '12px 16px',
                border: '1px solid #ddd',
                borderRadius: '24px',
                fontSize: '14px',
                outline: 'none'
              }}
            />
            <button
              onClick={handleSendMessage}
              disabled={isLoading || !inputMessage.trim()}
              style={{
                marginLeft: '8px',
                padding: '12px 20px',
                backgroundColor: inputMessage.trim() && !isLoading ? '#1976d2' : '#ccc',
                color: 'white',
                border: 'none',
                borderRadius: '24px',
                cursor: inputMessage.trim() && !isLoading ? 'pointer' : 'not-allowed'
              }}
            >
              Send
            </button>
          </div>
        </div>
      )}
    </div>
  );
};

export default RAGChatbot;