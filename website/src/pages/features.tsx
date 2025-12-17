import React from 'react';
import Layout from '@theme/Layout';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';

const FeaturesPage: React.FC = () => {
  const { siteConfig } = useDocusaurusContext();

  return (
    <Layout
      title={`Features | ${siteConfig.title}`}
      description="Explore the interactive features of our AI-Native Textbooks">
      <main className="container margin-vert--lg">
        <div className="row">
          <div className="col col--12">
            <header className="text--center margin-bottom--xl">
              <h1>Interactive Features</h1>
              <p className="hero__subtitle">
                Enhance your learning experience with our AI-powered features
              </p>
            </header>

            <div className="features-grid">
              <div className="feature-card">
                <div className="feature-icon">🤖</div>
                <h3>RAG-Powered Chatbot</h3>
                <p>
                  Get instant answers to your questions about Physical AI and Humanoid Robotics.
                  Our Retrieval-Augmented Generation (RAG) chatbot provides context-aware responses
                  based on the textbook content you're currently reading.
                </p>
                <div className="feature-actions">
                  <Link className="button button--primary" to="/textbooks/interactive/rag-chatbot">
                    Learn More
                  </Link>
                </div>
              </div>

              <div className="feature-card">
                <div className="feature-icon">👤</div>
                <h3>Personalized Content</h3>
                <p>
                  Customize your learning experience based on your background in software and hardware.
                  The content adapts to your technical level, providing more detailed explanations
                  for beginners or advanced insights for experienced practitioners.
                </p>
                <div className="feature-actions">
                  <Link className="button button--primary" to="/textbooks/interactive/personalization">
                    Learn More
                  </Link>
                </div>
              </div>

              <div className="feature-card">
                <div className="feature-icon">🇵🇰</div>
                <h3>Urdu Translation</h3>
                <p>
                  Access textbook content in Urdu to enhance understanding and inclusivity.
                  Our AI-powered translation preserves technical accuracy while making
                  complex concepts accessible in your preferred language.
                </p>
                <div className="feature-actions">
                  <Link className="button button--primary" to="/textbooks/interactive/translation">
                    Learn More
                  </Link>
                </div>
              </div>

              <div className="feature-card">
                <div className="feature-icon">📚</div>
                <h3>Interactive Elements</h3>
                <p>
                  Engage with interactive diagrams, simulations, and exercises that bring
                  Physical AI and Humanoid Robotics concepts to life. Test your understanding
                  with real-time feedback.
                </p>
                <div className="feature-actions">
                  <Link className="button button--primary" to="/textbooks/interactive/rag-chatbot">
                    Learn More
                  </Link>
                </div>
              </div>
            </div>

            <section className="feature-details">
              <h2>How It Works</h2>
              <div className="feature-steps">
                <div className="step">
                  <h3>1. Context-Aware Assistance</h3>
                  <p>
                    Our chatbot understands exactly what you're reading and provides relevant answers
                    based on the current textbook section. No more generic responses - just targeted
                    help for your specific learning needs.
                  </p>
                </div>

                <div className="step">
                  <h3>2. Adaptive Learning</h3>
                  <p>
                    Based on your background information (captured during registration),
                    the system tailors explanations, examples, and complexity to match your
                    experience level in robotics, programming, and mathematics.
                  </p>
                </div>

                <div className="step">
                  <h3>3. Multilingual Support</h3>
                  <p>
                    Complex technical concepts are made accessible through accurate translation
                    that maintains the integrity of scientific and engineering terminology.
                  </p>
                </div>

                <div className="step">
                  <h3>4. Real-time Interaction</h3>
                  <p>
                    All features work in real-time as you navigate through the textbook,
                    providing a seamless learning experience that adapts to your pace and preferences.
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

export default FeaturesPage;