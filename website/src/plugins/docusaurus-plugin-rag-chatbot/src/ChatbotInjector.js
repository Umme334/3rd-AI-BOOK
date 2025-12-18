import React, { useEffect } from 'react';
import RAGChatbot from '@site/src/components/chatbot/RAGChatbot';

const ChatbotInjector = () => {
  useEffect(() => {
    // Only inject the chatbot on textbook pages (those that contain textbook content)
    const isTextbookPage = window.location.pathname.includes('/textbooks/') ||
                          window.location.pathname.includes('/docs/') ||
                          window.location.pathname.includes('/chapter-');

    if (isTextbookPage) {
      // Create a container for the chatbot
      const chatbotContainer = document.createElement('div');
      chatbotContainer.id = 'rag-chatbot-container';
      chatbotContainer.style.position = 'fixed';
      chatbotContainer.style.bottom = '20px';
      chatbotContainer.style.right = '20px';
      chatbotContainer.style.zIndex = '1000';
      chatbotContainer.style.display = 'block';

      document.body.appendChild(chatbotContainer);

      // Render the chatbot component
      const renderChatbot = () => {
        const reactRoot = require('react-dom/client');
        const chatbotElement = React.createElement(RAGChatbot, {
          textbookId: 'physical-ai-textbook',
          chapterId: window.location.pathname.split('/').pop(), // Extract chapter ID from URL
          sectionTitle: document.title
        });

        const root = reactRoot.createRoot(chatbotContainer);
        root.render(chatbotElement);
      };

      // Delay rendering to ensure DOM is ready
      setTimeout(renderChatbot, 100);

      // Cleanup function
      return () => {
        if (chatbotContainer && chatbotContainer.parentNode) {
          chatbotContainer.parentNode.removeChild(chatbotContainer);
        }
      };
    }
  }, []);

  return null;
};

export default ChatbotInjector;