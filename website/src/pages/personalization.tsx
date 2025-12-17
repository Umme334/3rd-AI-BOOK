import React from 'react';
import Layout from '@theme/Layout';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';

const PersonalizationPage: React.FC = () => {
  const { siteConfig } = useDocusaurusContext();

  return (
    <Layout
      title={`Personalization | ${siteConfig.title}`}
      description="Personalized learning experience based on your background in Physical AI and Humanoid Robotics">
      <main className="container margin-vert--lg">
        <div className="row">
          <div className="col col--12">
            <header className="text--center margin-bottom--xl">
              <h1>Personalized Learning</h1>
              <p className="hero__subtitle">
                Content tailored to your software and hardware background for optimal learning
              </p>
            </header>

            <section className="feature-intro">
              <div className="row">
                <div className="col col--6">
                  <h2>Adaptive Content</h2>
                  <p>
                    Our personalization engine adapts textbook content based on your background
                    in software development, hardware experience, programming languages, and
                    robotics knowledge. Whether you're a beginner or an experienced practitioner,
                    the material adjusts to your level.
                  </p>
                  <h3>Background Categories:</h3>
                  <ul>
                    <li>Software Experience Level</li>
                    <li>Hardware Platform Familiarity</li>
                    <li>Programming Language Proficiency</li>
                    <li>Robotics Project Experience</li>
                    <li>Mathematical Background</li>
                  </ul>
                </div>
                <div className="col col--6">
                  <div className="personalization-demo">
                    <h3>Personalization Preview</h3>
                    <div className="content-example">
                      <h4>Standard Content:</h4>
                      <p>
                        "The control system uses a PID controller to regulate the joint positions.
                        The mathematical model involves differential equations that describe the
                        system dynamics."
                      </p>
                      <h4>Personalized for Hardware Expert:</h4>
                      <p>
                        "The control system uses a PID controller optimized for the specific
                        servo characteristics of your hardware platform. The tuning parameters
                        account for the motor response time and mechanical constraints."
                      </p>
                      <h4>Personalized for Software Developer:</h4>
                      <p>
                        "The control system implements a PID algorithm with parameters that
                        can be adjusted for different response characteristics. The implementation
                        handles sensor feedback and actuator commands in real-time."
                      </p>
                    </div>
                  </div>
                </div>
              </div>
            </section>

            <section className="feature-benefits">
              <h2>How Personalization Works</h2>
              <div className="row">
                <div className="col col--4">
                  <div className="benefit-card">
                    <h3>1. Background Assessment</h3>
                    <p>
                      During registration, you provide information about your experience
                      in software, hardware, and robotics to establish your learning profile.
                    </p>
                  </div>
                </div>
                <div className="col col--4">
                  <div className="benefit-card">
                    <h3>2. Dynamic Adaptation</h3>
                    <p>
                      Content is dynamically adjusted as you navigate through textbooks,
                      providing the right level of detail and explanation for your background.
                    </p>
                  </div>
                </div>
                <div className="col col--4">
                  <div className="benefit-card">
                    <h3>3. Continuous Learning</h3>
                    <p>
                      Your preferences and interaction patterns help refine the personalization
                      over time for an increasingly tailored experience.
                    </p>
                  </div>
                </div>
              </div>
            </section>

            <section className="personalization-options">
              <h2>Personalization Settings</h2>
              <div className="row">
                <div className="col col--6">
                  <div className="setting-card">
                    <h3>Technical Depth</h3>
                    <p>Adjust the level of technical detail based on your expertise:</p>
                    <ul>
                      <li>Beginner: Conceptual explanations with analogies</li>
                      <li>Intermediate: Technical details with examples</li>
                      <li>Advanced: Deep mathematical and implementation details</li>
                    </ul>
                  </div>
                </div>
                <div className="col col--6">
                  <div className="setting-card">
                    <h3>Platform Focus</h3>
                    <p>Emphasize content relevant to your hardware/software platform:</p>
                    <ul>
                      <li>ROS 2: Robot Operating System examples and tutorials</li>
                      <li>Gazebo: Simulation-focused content and examples</li>
                      <li>NVIDIA Isaac: GPU-accelerated AI and robotics content</li>
                      <li>Custom: Bring your own hardware platform</li>
                    </ul>
                  </div>
                </div>
              </div>
            </section>

            <section className="integration-features">
              <h2>Integration with Learning</h2>
              <div className="integration-grid">
                <div className="integration-item">
                  <h3>Context-Aware Examples</h3>
                  <p>
                    Examples and code snippets are tailored to your preferred programming
                    languages and development environments.
                  </p>
                </div>
                <div className="integration-item">
                  <h3>Difficulty Adjustment</h3>
                  <p>
                    Exercise complexity and problem sets adapt to challenge you appropriately
                    based on your demonstrated knowledge.
                  </p>
                </div>
                <div className="integration-item">
                  <h3>Resource Recommendations</h3>
                  <p>
                    Additional reading materials and resources are suggested based on
                    gaps in your knowledge profile.
                  </p>
                </div>
                <div className="integration-item">
                  <h3>Progress Tracking</h3>
                  <p>
                    Learning progress is tracked in the context of your background,
                    highlighting areas where you've made significant improvement.
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

export default PersonalizationPage;