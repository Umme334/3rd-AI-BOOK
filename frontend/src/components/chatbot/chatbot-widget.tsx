import React, { useState, useEffect, useRef } from 'react';

import {
  ChatbotQueryRequest,
  ChatbotResponse,
  ChatMessage as ChatbotMessage,
  ChatbotSession
} from '../../types/chatbot';

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
  const messagesEndRef = useRef<null | HTMLDivElement>(null);

  // Function to simulate sending a message to the backend
  const sendMessage = async (query: string) => {
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

      // Prepare the query request
      const request: ChatbotQueryRequest = {
        query,
        textbook_id: textbookId,
        session_id: currentSessionId
      };

      // In a real implementation, this would call the backend API
      // For now, we'll simulate a response
      setTimeout(() => {
        const botResponse: ChatbotMessage = {
          id: `msg-${Date.now()}-bot`,
          role: 'assistant',
          content: `This is a simulated response to your query: "${query}". In a real implementation, this would come from the RAG chatbot system based on the Physical AI & Humanoid Robotics textbook content.`,
          timestamp: new Date().toISOString()
        };

        setMessages(prev => [...prev, botResponse]);
        setIsLoading(false);
        setInputMessage('');
      }, 1000);

    } catch (error) {
      console.error('Error sending message:', error);
      setIsLoading(false);
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

  return (
    <div className="chatbot-widget">
      {isOpen ? (
        <div className="chatbot-window">
          <div className="chatbot-header">
            <h3>Physical AI Assistant</h3>
            <button onClick={() => setIsOpen(false)}>×</button>
          </div>
          <div className="chatbot-messages">
            {messages.length === 0 ? (
              <div className="welcome-message">
                <p>Hello! I'm your Physical AI & Humanoid Robotics assistant.</p>
                <p>Ask me questions about the textbook content, and I'll provide answers based on the material.</p>
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
              placeholder="Ask a question about the textbook..."
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
export { ChatbotWidget };