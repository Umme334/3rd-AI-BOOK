import React from 'react';
import Layout from '@theme/Layout';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';

const RagChatbotPage: React.FC = () => {
  const { siteConfig } = useDocusaurusContext();

  return (
    <Layout
      title={`RAG Chatbot | ${siteConfig.title}`}
      description="Interactive RAG-powered chatbot for Physical AI and Humanoid Robotics">
      <main className="container margin-vert--lg">
        <div className="row">
          <div className="col col--12">
            <header className="text--center margin-bottom--xl">
              <h1>RAG-Powered Chatbot</h1>
              <p className="hero__subtitle">
                Get instant, context-aware answers to your questions about Physical AI and Humanoid Robotics
              </p>
            </header>

            <section className="feature-intro">
              <div className="row">
                <div className="col col--6">
                  <h2>How It Works</h2>
                  <p>
                    Our Retrieval-Augmented Generation (RAG) chatbot combines the power of large language models
                    with your textbook content to provide accurate, context-aware responses. Instead of generating
                    generic answers, the chatbot retrieves relevant information from the specific textbook sections
                    you're reading.
                  </p>
                  <ul>
                    <li>Context-aware responses based on current textbook section</li>
                    <li>Accurate information sourced directly from textbook content</li>
                    <li>Real-time interaction as you navigate through content</li>
                    <li>Technical accuracy maintained for complex robotics concepts</li>
                  </ul>
                </div>
                <div className="col col--6">
                  <div className="chatbot-demo-placeholder">
                    <h3>Interactive Demo</h3>
                    <p>
                      Try asking questions about the content you're reading, and see how the chatbot
                      provides answers based on the specific context of your textbook.
                    </p>
                    <div className="chatbot-placeholder">
                      <div className="message bot">
                        <div className="message-content">
                          Hello! I'm your Physical AI & Humanoid Robotics assistant.
                          Ask me questions about the content you're reading, and I'll provide
                          answers based on the textbook material.
                        </div>
                      </div>
                    </div>
                  </div>
                </div>
              </div>
            </section>

            <section className="feature-benefits">
              <h2>Key Benefits</h2>
              <div className="row">
                <div className="col col--4">
                  <div className="benefit-card">
                    <h3>🎯 Contextual Understanding</h3>
                    <p>
                      The chatbot understands exactly what you're reading and provides relevant answers
                      based on the current textbook section, chapter, or concept.
                    </p>
                  </div>
                </div>
                <div className="col col--4">
                  <div className="benefit-card">
                    <h3>📚 Accurate Information</h3>
                    <p>
                      All responses are grounded in your textbook content, ensuring technical accuracy
                      and consistency with the material you're studying.
                    </p>
                  </div>
                </div>
                <div className="col col--4">
                  <div className="benefit-card">
                    <h3>⚡ Instant Answers</h3>
                    <p>
                      Get immediate responses to your questions without flipping through pages
                      or searching through documentation.
                    </p>
                  </div>
                </div>
              </div>
            </section>

            <section className="integration-examples">
              <h2>Integration Examples</h2>
              <div className="example-cards">
                <div className="example-card">
                  <h3>ROS 2 Questions</h3>
                  <p>
                    "Explain the difference between ROS 2 topics and services in the context of humanoid robot control"
                  </p>
                </div>
                <div className="example-card">
                  <h3>Kinematics Queries</h3>
                  <p>
                    "How does inverse kinematics work for a 6-DOF humanoid arm in the current chapter?"
                  </p>
                </div>
                <div className="example-card">
                  <h3>Control Theory</h3>
                  <p>
                    "What are the stability considerations for the balance controller described in this section?"
                  </p>
                </div>
              </div>
            </section>
          </div>
        </div>
      </main>
    </Layout>
  );
};

export default RagChatbotPage;