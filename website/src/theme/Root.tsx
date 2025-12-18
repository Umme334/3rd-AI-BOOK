import React, { useEffect, useState } from 'react';
import ChatbotWidget from '../components/chatbot/ChatbotWidget';

// Default textbook ID - can be overridden by individual pages
const DEFAULT_TEXTBOOK_ID = 'physical-ai-textbook';

const Root = ({ children }) => {
  const [textbookId, setTextbookId] = useState(DEFAULT_TEXTBOOK_ID);

  // Attempt to get textbook ID from URL or metadata
  useEffect(() => {
    // In a real implementation, you could extract textbook ID from the URL
    // or from page metadata, but for now we'll use a default
    const path = window.location.pathname;
    if (path.includes('textbooks')) {
      // Extract textbook identifier from path
      const pathParts = path.split('/');
      const textbookIndex = pathParts.indexOf('textbooks');
      if (textbookIndex !== -1 && pathParts[textbookIndex + 1]) {
        setTextbookId(pathParts[textbookIndex + 1]);
      }
    }
  }, []);

  return (
    <>
      {children}
      <ChatbotWidget textbookId={textbookId} />
    </>
  );
};

export default Root;