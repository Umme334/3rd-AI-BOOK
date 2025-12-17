import React, { useState } from 'react';

import { TextbookCreateRequest, DifficultyLevel } from '../types/textbook';

interface TextbookGeneratorProps {
  onGenerate: (request: TextbookCreateRequest) => void;
}

const TextbookGenerator: React.FC<TextbookGeneratorProps> = ({ onGenerate }) => {
  const [title, setTitle] = useState('');
  const [subject, setSubject] = useState('');
  const [difficulty, setDifficulty] = useState<DifficultyLevel>('beginner');
  const [targetAudience, setTargetAudience] = useState('');
  const [chapterCount, setChapterCount] = useState(5);
  const [includeQuizzes, setIncludeQuizzes] = useState(true);
  const [includeSummaries, setIncludeSummaries] = useState(true);
  // Physical AI specific fields
  const [courseModule, setCourseModule] = useState('');
  const [hardwareRequirements, setHardwareRequirements] = useState('');
  const [enablePersonalization, setEnablePersonalization] = useState(false);
  const [enableTranslation, setEnableTranslation] = useState(false);

  const handleSubmit = (e: React.FormEvent) => {
    e.preventDefault();

    const request: TextbookCreateRequest = {
      title,
      subject,
      difficulty,
      target_audience: targetAudience,
      chapter_count: chapterCount,
      include_quizzes: includeQuizzes,
      include_summaries: includeSummaries,
      // Physical AI specific attributes
      course_module: courseModule || undefined,
      hardware_requirements: hardwareRequirements ? hardwareRequirements.split(',').map(item => item.trim()) : undefined,
      enable_personalization: enablePersonalization,
      enable_translation: enableTranslation,
    };

    onGenerate(request);
  };

  return (
    <div className="textbook-generator">
      <h2>Generate New Physical AI & Humanoid Robotics Textbook</h2>
      <form onSubmit={handleSubmit}>
        <div className="form-group">
          <label htmlFor="title">Title:</label>
          <input
            type="text"
            id="title"
            value={title}
            onChange={(e) => setTitle(e.target.value)}
            required
            placeholder="e.g., Introduction to Physical AI and Humanoid Robotics"
          />
        </div>

        <div className="form-group">
          <label htmlFor="subject">Subject:</label>
          <input
            type="text"
            id="subject"
            value={subject}
            onChange={(e) => setSubject(e.target.value)}
            required
            placeholder="e.g., Physical AI and Humanoid Robotics"
          />
        </div>

        <div className="form-group">
          <label htmlFor="difficulty">Difficulty:</label>
          <select
            id="difficulty"
            value={difficulty}
            onChange={(e) => setDifficulty(e.target.value as DifficultyLevel)}
          >
            <option value="beginner">Beginner</option>
            <option value="intermediate">Intermediate</option>
            <option value="advanced">Advanced</option>
          </select>
        </div>

        <div className="form-group">
          <label htmlFor="courseModule">Course Module:</label>
          <select
            id="courseModule"
            value={courseModule}
            onChange={(e) => setCourseModule(e.target.value)}
          >
            <option value="">Select a module</option>
            <option value="Introduction">Introduction to Physical AI and Embodied Intelligence</option>
            <option value="ROS 2">ROS 2: The Robotic Nervous System</option>
            <option value="Gazebo">Robot Simulation with Gazebo</option>
            <option value="NVIDIA Isaac">NVIDIA Isaac Platform</option>
            <option value="Vision-Language-Action">Vision-Language-Action Integration</option>
          </select>
        </div>

        <div className="form-group">
          <label htmlFor="targetAudience">Target Audience:</label>
          <input
            type="text"
            id="targetAudience"
            value={targetAudience}
            onChange={(e) => setTargetAudience(e.target.value)}
            placeholder="e.g., Undergraduate students, Robotics enthusiasts"
          />
        </div>

        <div className="form-group">
          <label htmlFor="hardwareRequirements">Hardware Requirements:</label>
          <input
            type="text"
            id="hardwareRequirements"
            value={hardwareRequirements}
            onChange={(e) => setHardwareRequirements(e.target.value)}
            placeholder="e.g., RTX 4070 Ti, Jetson Orin Nano, RealSense D435i"
          />
          <small>Separate multiple requirements with commas</small>
        </div>

        <div className="form-group">
          <label htmlFor="chapterCount">Number of Chapters (1-20):</label>
          <input
            type="number"
            id="chapterCount"
            min="1"
            max="20"
            value={chapterCount}
            onChange={(e) => setChapterCount(parseInt(e.target.value))}
          />
        </div>

        <div className="form-group">
          <label>
            <input
              type="checkbox"
              checked={includeQuizzes}
              onChange={(e) => setIncludeQuizzes(e.target.checked)}
            />
            Include Quizzes
          </label>
        </div>

        <div className="form-group">
          <label>
            <input
              type="checkbox"
              checked={includeSummaries}
              onChange={(e) => setIncludeSummaries(e.target.checked)}
            />
            Include Summaries
          </label>
        </div>

        <div className="form-group">
          <label>
            <input
              type="checkbox"
              checked={enablePersonalization}
              onChange={(e) => setEnablePersonalization(e.target.checked)}
            />
            Enable Content Personalization
          </label>
        </div>

        <div className="form-group">
          <label>
            <input
              type="checkbox"
              checked={enableTranslation}
              onChange={(e) => setEnableTranslation(e.target.checked)}
            />
            Enable Urdu Translation
          </label>
        </div>

        <button type="submit">Generate Textbook</button>
      </form>
    </div>
  );
};

export default TextbookGenerator;