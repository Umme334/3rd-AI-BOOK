import React from 'react';
import Layout from '@theme/Layout';
import ChatbotWidget from './ChatbotWidget';

interface TextbookLayoutProps {
  children: React.ReactNode;
  title?: string;
  description?: string;
  textbookId: string;
}

const TextbookLayout: React.FC<TextbookLayoutProps> = ({
  children,
  title = 'Physical AI & Humanoid Robotics Textbook',
  description = 'Interactive textbook with RAG chatbot',
  textbookId
}) => {
  return (
    <Layout title={title} description={description}>
      <ChatbotWidget textbookId={textbookId} />
      <main>{children}</main>
    </Layout>
  );
};

export default TextbookLayout;