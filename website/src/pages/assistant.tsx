import React from 'react';
import Layout from '@theme/Layout';
import RAGChatbot from '../components/chatbot/RAGChatbot';

const AssistantPage = () => {
  return (
    <Layout
      title="Physical AI Assistant"
      description="Interactive AI assistant for Physical AI and Humanoid Robotics">
      <main className="container margin-vert--lg">
        <div className="row">
          <div className="col col--12">
            <header className="text--center margin-bottom--xl">
              <h1>Physical AI & Humanoid Robotics Assistant</h1>
              <p className="hero__subtitle">
                Your interactive assistant for Physical AI and Humanoid Robotics concepts
              </p>
            </header>

            <section className="margin-vert--lg">
              <div style={{
                height: '70vh',
                border: '1px solid #ddd',
                borderRadius: '8px',
                padding: '20px'
              }}>
                <RAGChatbot textbookId="physical-ai-textbook" />
              </div>
            </section>

            <section className="margin-vert--lg">
              <h2>How to Use the Assistant</h2>
              <div className="row">
                <div className="col col--4">
                  <h3>🎯 Ask Questions</h3>
                  <p>Ask specific questions about Physical AI, Humanoid Robotics, ROS 2, control systems, and more.</p>
                </div>
                <div className="col col--4">
                  <h3>📚 Get Contextual Answers</h3>
                  <p>The assistant provides answers based on the textbook content you're studying.</p>
                </div>
                <div className="col col--4">
                  <h3>⚡ Instant Help</h3>
                  <p>Get immediate responses without flipping through pages or searching documentation.</p>
                </div>
              </div>
            </section>
          </div>
        </div>
      </main>
    </Layout>
  );
};

export default AssistantPage;