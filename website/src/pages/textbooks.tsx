import React, { useState, useEffect } from 'react';
import Layout from '@theme/Layout';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import useBaseUrl from '@docusaurus/useBaseUrl';

interface Textbook {
  id: string;
  title: string;
  description: string;
  author: string;
  publishedDate: string;
  chapters: Chapter[];
  thumbnail?: string;
}

interface Chapter {
  id: string;
  title: string;
  description: string;
  sections: Section[];
  estimatedReadingTime: number;
}

interface Section {
  id: string;
  title: string;
  url: string;
}

const TextbooksPage: React.FC = () => {
  const { siteConfig } = useDocusaurusContext();
  const [textbooks, setTextbooks] = useState<Textbook[]>([]);
  const [loading, setLoading] = useState(true);
  const [selectedTextbook, setSelectedTextbook] = useState<string | null>(null);

  // Mock data for textbooks - in a real implementation, this would come from an API
  useEffect(() => {
    // Simulate API call to fetch textbooks
    setTimeout(() => {
      const mockTextbooks: Textbook[] = [
        {
          id: 'phy-ai-humanoids-001',
          title: 'Introduction to Physical AI and Humanoid Robotics',
          description: 'A comprehensive guide to the fundamentals of Physical AI and Humanoid Robotics, covering both theoretical concepts and practical applications.',
          author: 'Dr. Jane Smith',
          publishedDate: '2025-01-15',
          thumbnail: '/img/textbook-physical-ai.jpg',
          chapters: [
            {
              id: 'ch-1',
              title: 'Introduction to Physical AI',
              description: 'Understanding the basics of Physical AI and its applications',
              estimatedReadingTime: 25,
              sections: [
                { id: 'sec-1-1', title: 'What is Physical AI?', url: '/textbooks/phy-ai-humanoids-001/ch-1/sec-1-1' },
                { id: 'sec-1-2', title: 'Historical Context', url: '/textbooks/phy-ai-humanoids-001/ch-1/sec-1-2' },
                { id: 'sec-1-3', title: 'Key Challenges', url: '/textbooks/phy-ai-humanoids-001/ch-1/sec-1-3' },
              ]
            },
            {
              id: 'ch-2',
              title: 'Humanoid Robotics Fundamentals',
              description: 'Core principles of humanoid robot design and control',
              estimatedReadingTime: 30,
              sections: [
                { id: 'sec-2-1', title: 'Kinematics', url: '/textbooks/phy-ai-humanoids-001/ch-2/sec-2-1' },
                { id: 'sec-2-2', title: 'Dynamics', url: '/textbooks/phy-ai-humanoids-001/ch-2/sec-2-2' },
                { id: 'sec-2-3', title: 'Control Systems', url: '/textbooks/phy-ai-humanoids-001/ch-2/sec-2-3' },
              ]
            },
            {
              id: 'ch-3',
              title: 'ROS 2 Integration',
              description: 'Using ROS 2 for humanoid robot development',
              estimatedReadingTime: 35,
              sections: [
                { id: 'sec-3-1', title: 'ROS 2 Basics', url: '/textbooks/phy-ai-humanoids-001/ch-3/sec-3-1' },
                { id: 'sec-3-2', title: 'Navigation Stack', url: '/textbooks/phy-ai-humanoids-001/ch-3/sec-3-2' },
                { id: 'sec-3-3', title: 'Custom Nodes', url: '/textbooks/phy-ai-humanoids-001/ch-3/sec-3-3' },
              ]
            }
          ]
        },
        {
          id: 'adv-humanoid-control-002',
          title: 'Advanced Control Techniques for Humanoid Robots',
          description: 'Deep dive into advanced control algorithms and methodologies for humanoid robots, including balance control and gait generation.',
          author: 'Prof. John Doe',
          publishedDate: '2025-02-20',
          thumbnail: '/img/textbook-advanced-control.jpg',
          chapters: [
            {
              id: 'ch-1',
              title: 'Balance Control',
              description: 'Maintaining stability in humanoid robots',
              estimatedReadingTime: 40,
              sections: [
                { id: 'sec-1-1', title: 'Zero Moment Point', url: '/textbooks/adv-humanoid-control-002/ch-1/sec-1-1' },
                { id: 'sec-1-2', title: 'Capture Point', url: '/textbooks/adv-humanoid-control-002/ch-1/sec-1-2' },
                { id: 'sec-1-3', title: 'Balance Controllers', url: '/textbooks/adv-humanoid-control-002/ch-1/sec-1-3' },
              ]
            }
          ]
        }
      ];

      setTextbooks(mockTextbooks);
      setLoading(false);
    }, 500);
  }, []);

  const toggleTextbookDetails = (id: string) => {
    setSelectedTextbook(selectedTextbook === id ? null : id);
  };

  return (
    <Layout
      title={`Textbooks | ${siteConfig.title}`}
      description="Browse our collection of AI-native textbooks on Physical AI and Humanoid Robotics">
      <main className="container margin-vert--lg">
        <div className="row">
          <div className="col col--12">
            <header className="text--center margin-bottom--xl">
              <h1>AI-Native Textbooks</h1>
              <p className="hero__subtitle">
                Interactive textbooks enhanced with RAG chatbots, personalization, and Urdu translation
              </p>
            </header>

            {loading ? (
              <div className="text--center padding--lg">
                <div className="loading-spinner">Loading textbooks...</div>
              </div>
            ) : (
              <div className="textbooks-grid">
                {textbooks.map((textbook) => (
                  <div key={textbook.id} className="textbook-card">
                    <div className="textbook-header">
                      <h2 className="textbook-title">
                        <Link to={`/docs/${textbook.id}`}>
                          {textbook.title}
                        </Link>
                      </h2>
                      <div className="textbook-meta">
                        <span className="textbook-author">by {textbook.author}</span>
                        <span className="textbook-date">{textbook.publishedDate}</span>
                      </div>
                    </div>

                    <div className="textbook-body">
                      <p className="textbook-description">{textbook.description}</p>

                      <div className="textbook-stats">
                        <span className="stat">
                          <i className="fa fa-book"></i> {textbook.chapters.length} Chapters
                        </span>
                        <span className="stat">
                          <i className="fa fa-clock-o"></i> ~{textbook.chapters.reduce((sum, ch) => sum + ch.estimatedReadingTime, 0)} min read
                        </span>
                      </div>

                      <div className="textbook-actions">
                        <Link
                          className="button button--primary button--outline"
                          to={`/docs/${textbook.id}`}
                        >
                          Browse Textbook
                        </Link>
                        <button
                          className="button button--secondary"
                          onClick={() => toggleTextbookDetails(textbook.id)}
                        >
                          {selectedTextbook === textbook.id ? 'Hide Details' : 'Show Details'}
                        </button>
                      </div>
                    </div>

                    {selectedTextbook === textbook.id && (
                      <div className="textbook-details">
                        <h3>Chapters:</h3>
                        <ul className="chapter-list">
                          {textbook.chapters.map((chapter) => (
                            <li key={chapter.id} className="chapter-item">
                              <div className="chapter-header">
                                <h4>
                                  <Link to={chapter.sections[0]?.url || `/docs/${textbook.id}/${chapter.id}`}>
                                    {chapter.title}
                                  </Link>
                                </h4>
                                <span className="chapter-time">{chapter.estimatedReadingTime} min</span>
                              </div>
                              <p className="chapter-description">{chapter.description}</p>

                              <div className="sections-list">
                                <strong>Sections:</strong>
                                <ul>
                                  {chapter.sections.map((section) => (
                                    <li key={section.id}>
                                      <Link to={section.url}>{section.title}</Link>
                                    </li>
                                  ))}
                                </ul>
                              </div>
                            </li>
                          ))}
                        </ul>
                      </div>
                    )}
                  </div>
                ))}
              </div>
            )}
          </div>
        </div>
      </main>
    </Layout>
  );
};

export default TextbooksPage;