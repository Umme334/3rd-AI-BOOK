import React, { useState } from 'react';
import { Container, Paper, Tabs, Tab, Box } from '@mui/material';
import InputForm from '../components/textbook-generator/input-form';
import StructureCustomizer from '../components/textbook-generator/structure-customizer';
import TextbookPreview from '../components/preview/textbook-preview';
import ExportOptions from '../components/export/export-options';
import ProgressTracker from '../components/progress/progress-tracker';

interface TextbookData {
  id?: string;
  title?: string;
  subject?: string;
  difficulty?: string;
  targetAudience?: string;
  chapters?: any[];
  chaptersCount?: number;
  structure?: any[];
}

const TextbookGenerationPage: React.FC = () => {
  const [activeTab, setActiveTab] = useState(0);
  const [textbookData, setTextbookData] = useState<TextbookData>({});
  const [isGenerating, setIsGenerating] = useState(false);
  const [generationProgress, setGenerationProgress] = useState(0);
  const [generatedTextbook, setGeneratedTextbook] = useState<any>(null);

  const handleFormSubmit = (data: any) => {
    const newTextbookData = {
      id: `textbook-${Date.now()}`,
      title: `${data.subject} Textbook`,
      subject: data.subject,
      difficulty: data.difficulty,
      targetAudience: data.targetAudience,
      chaptersCount: data.chapter_count || 5, // Using the correct field from the form
      selectedModules: data.selectedModules
    };
    setTextbookData(newTextbookData);
    setIsGenerating(true);
    setActiveTab(2); // Switch to preview tab
  };

  const handleStructureSave = (structure: any) => {
    setTextbookData(prev => ({
      ...prev,
      structure
    }));
  };

  const handleGenerationComplete = () => {
    setIsGenerating(false);
    setGeneratedTextbook({
      id: textbookData.id,
      title: textbookData.title,
      subject: textbookData.subject,
      difficulty: textbookData.difficulty,
      targetAudience: textbookData.targetAudience,
      chapters: Array.from({ length: textbookData.chaptersCount || 5 }, (_, i) => ({
        id: `chapter-${i + 1}`,
        title: `Chapter ${i + 1}: ${['Introduction', 'ROS 2', 'Gazebo', 'NVIDIA Isaac', 'Vision-Language-Action'][i] || `Topic ${i + 1}`}`,
        sections: Array.from({ length: 3 }, (_, j) => ({
          id: `section-${i + 1}-${j + 1}`,
          title: `Section ${j + 1}: ${['Overview', 'Implementation', 'Examples'][j] || `Subtopic ${j + 1}`}`,
          content: `This is the content for ${['Overview', 'Implementation', 'Examples'][j] || `Subtopic ${j + 1}`}. It covers the fundamental concepts of Physical AI and Humanoid Robotics in the context of ${['Introduction', 'ROS 2', 'Gazebo', 'NVIDIA Isaac', 'Vision-Language-Action'][i] || `Topic ${i + 1}`}.`
        }))
      }))
    });
  };

  const handleExport = (format: string, url: string) => {
    alert(`Exported as ${format} format. Download URL: ${url}`);
  };

  const handleTabChange = (event: React.SyntheticEvent, newValue: number) => {
    setActiveTab(newValue);
  };

  return (
    <Container maxWidth="lg" sx={{ py: 3 }}>
      <Paper sx={{ p: 3 }}>
        <Tabs
          value={activeTab}
          onChange={handleTabChange}
          variant="scrollable"
          scrollButtons="auto"
        >
          <Tab label="Create Textbook" />
          <Tab label="Customize Structure" />
          <Tab label="Preview" />
          <Tab label="Export" />
        </Tabs>

        <Box sx={{ mt: 3 }}>
          {activeTab === 0 && (
            <InputForm onSubmit={handleFormSubmit} />
          )}

          {activeTab === 1 && (
            <StructureCustomizer
              initialStructure={textbookData.structure || Array.from({ length: textbookData.chaptersCount || 5 }, (_, i) => ({
                id: `chapter-${i + 1}`,
                title: `Chapter ${i + 1}: ${['Introduction', 'ROS 2', 'Gazebo', 'NVIDIA Isaac', 'Vision-Language-Action'][i] || `Topic ${i + 1}`}`,
                sections: Array.from({ length: 3 }, (_, j) => ({
                  id: `section-${i + 1}-${j + 1}`,
                  title: `Section ${j + 1}: ${['Overview', 'Implementation', 'Examples'][j] || `Subtopic ${j + 1}`}`,
                }))
              }))}
              onSave={handleStructureSave}
            />
          )}

          {activeTab === 2 && (
            <Box>
              {isGenerating ? (
                <ProgressTracker
                  textbookId={textbookData.id || 'new'}
                  onProgressUpdate={setGenerationProgress}
                  onComplete={handleGenerationComplete}
                />
              ) : (
                <TextbookPreview
                  textbook={generatedTextbook}
                  onExport={() => setActiveTab(3)}
                  onRegenerate={() => {
                    setIsGenerating(true);
                    setActiveTab(2);
                  }}
                  isLoading={isGenerating}
                />
              )}
            </Box>
          )}

          {activeTab === 3 && generatedTextbook && (
            <ExportOptions
              textbookId={generatedTextbook.id}
              onExportComplete={handleExport}
            />
          )}
        </Box>
      </Paper>
    </Container>
  );
};

export default TextbookGenerationPage;