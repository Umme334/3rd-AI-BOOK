import React from 'react';
import Layout from '@theme/Layout';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';

const TranslationPage: React.FC = () => {
  const { siteConfig } = useDocusaurusContext();

  return (
    <Layout
      title={`Urdu Translation | ${siteConfig.title}`}
      description="Urdu translation for Physical AI and Humanoid Robotics textbook content">
      <main className="container margin-vert--lg">
        <div className="row">
          <div className="col col--12">
            <header className="text--center margin-bottom--xl">
              <h1>Urdu Translation</h1>
              <p className="hero__subtitle">
                Access complex Physical AI and Humanoid Robotics concepts in Urdu for enhanced understanding
              </p>
            </header>

            <section className="feature-intro">
              <div className="row">
                <div className="col col--6">
                  <h2>Technical Translation</h2>
                  <p>
                    Our AI-powered translation system preserves the technical accuracy of complex
                    robotics and AI concepts while making them accessible in Urdu. Unlike generic
                    translation tools, our system understands domain-specific terminology and
                    maintains the integrity of scientific and engineering concepts.
                  </p>
                  <h3>Translation Features:</h3>
                  <ul>
                    <li>Technical terminology preservation</li>
                    <li>Context-aware translation for accuracy</li>
                    <li>Real-time translation as you read</li>
                    <li>Toggle between English and Urdu</li>
                    <li>Academic-level translation quality</li>
                  </ul>
                </div>
                <div className="col col--6">
                  <div className="translation-demo">
                    <h3>Translation Example</h3>
                    <div className="content-comparison">
                      <div className="content-pair">
                        <div className="original-content">
                          <h4>English:</h4>
                          <p>
                            "The humanoid robot's gait is controlled by a central pattern generator
                            that coordinates the rhythmic movement of the legs. This biological
                            inspiration allows for stable walking patterns on various terrains."
                          </p>
                        </div>
                        <div className="translated-content">
                          <h4>اردو:</h4>
                          <p>
                            "ہیومنوڈ روبوٹ کی چال کو ایک مرکزی پیٹرن جنریٹر کے ذریعے کنٹرول کیا جاتا ہے
                            جو بازوں کی مکمل حرکت کو مطابقت دیتا ہے۔ یہ حیاتیاتی حوصلہ افرائی مختلف
                            زمینوں پر مستحکم چلنے کے نمونے کی اجازت دیتا ہے۔"
                          </p>
                        </div>
                      </div>
                    </div>
                  </div>
                </div>
              </div>
            </section>

            <section className="feature-benefits">
              <h2>Benefits of Urdu Translation</h2>
              <div className="row">
                <div className="col col--4">
                  <div className="benefit-card">
                    <h3>📚 Enhanced Comprehension</h3>
                    <p>
                      Complex technical concepts become more accessible when presented in your
                      preferred language, leading to better understanding and retention.
                    </p>
                  </div>
                </div>
                <div className="col col--4">
                  <div className="benefit-card">
                    <h3>🎯 Technical Accuracy</h3>
                    <p>
                      Our specialized translation system maintains the precision of robotics
                      and AI terminology while making it understandable in Urdu.
                    </p>
                  </div>
                </div>
                <div className="col col--4">
                  <div className="benefit-card">
                    <h3>⚡ Real-time Access</h3>
                    <p>
                      Switch between English and Urdu instantly as you navigate through
                      textbook content without losing your place or context.
                    </p>
                  </div>
                </div>
              </div>
            </section>

            <section className="translation-process">
              <h2>How Translation Works</h2>
              <div className="process-steps">
                <div className="step">
                  <h3>1. AI-Powered Translation</h3>
                  <p>
                    Our system uses advanced AI models specifically trained on robotics and
                    AI literature to provide accurate translations of technical content.
                  </p>
                </div>
                <div className="step">
                  <h3>2. Context Preservation</h3>
                  <p>
                    The translation maintains the context of the textbook section, ensuring
                    that technical relationships and concepts remain clear.
                  </p>
                </div>
                <div className="step">
                  <h3>3. Quality Assurance</h3>
                  <p>
                    Technical terms are validated against established robotics and AI
                    terminology in Urdu to ensure consistency and accuracy.
                  </p>
                </div>
                <div className="step">
                  <h3>4. Seamless Integration</h3>
                  <p>
                    Translation is integrated directly into the textbook interface,
                    allowing for smooth switching between languages.
                  </p>
                </div>
              </div>
            </section>

            <section className="supported-content">
              <h2>Supported Content Types</h2>
              <div className="row">
                <div className="col col--3">
                  <div className="content-type">
                    <h4>Text</h4>
                    <p>Full paragraphs and explanations</p>
                  </div>
                </div>
                <div className="col col--3">
                  <div className="content-type">
                    <h4>Code</h4>
                    <p>Comments and documentation</p>
                  </div>
                </div>
                <div class="col col--3">
                  <div class="content-type">
                    <h4>Diagrams</h4>
                    <p>Captions and labels</p>
                  </div>
                </div>
                <div class="col col--3">
                  <div class="content-type">
                    <h4>Exercises</h4>
                    <p>Problems and questions</p>
                  </div>
                </div>
              </div>
            </section>

            <section class="accessibility-features">
              <h2>Accessibility Features</h2>
              <div class="features-grid">
                <div class="accessibility-item">
                  <h3>Toggle Interface</h3>
                  <p>
                    Easy-to-use toggle buttons allow switching between English and Urdu
                    without disrupting your reading flow.
                  </p>
                </div>
                <div class="accessibility-item">
                  <h3>Progress Preservation</h3>
                  <p>
                    Your reading progress is maintained across language switches,
                    so you never lose your place.
                  </p>
                </div>
                <div class="accessibility-item">
                  <h3>Terminology Glossary</h3>
                  <p>
                    Access a glossary of technical terms with both English and Urdu equivalents
                    for reference.
                  </p>
                </div>
                <div class="accessibility-item">
                  <h3>Voice Support</h3>
                  <p>
                    Urdu text can be read aloud using text-to-speech for enhanced accessibility.
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

export default TranslationPage;