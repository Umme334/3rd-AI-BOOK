import React from 'react';
import ChatbotWidget from './ChatbotWidget';

interface WithChatbotProps {
  children: React.ReactNode;
  textbookId: string;
}

const WithChatbot: React.FC<WithChatbotProps> = ({ children, textbookId }) => {
  return (
    <>
      {children}
      <ChatbotWidget textbookId={textbookId} />
    </>
  );
};

export default WithChatbot;