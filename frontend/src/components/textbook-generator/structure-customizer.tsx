import React, { useState } from 'react';
import { Box, Typography, TextField, Button, Accordion, AccordionSummary, AccordionDetails, IconButton, List, ListItem, ListItemText, ListItemSecondaryAction } from '@mui/material';
import ExpandMoreIcon from '@mui/icons-material/ExpandMore';
import AddIcon from '@mui/icons-material/Add';
import DeleteIcon from '@mui/icons-material/Delete';

interface Chapter {
  id: string;
  title: string;
  sections: Section[];
}

interface Section {
  id: string;
  title: string;
  contentTypes: string[];
}

interface StructureCustomizerProps {
  initialStructure?: Chapter[];
  onSave: (structure: Chapter[]) => void;
}

const StructureCustomizer: React.FC<StructureCustomizerProps> = ({ initialStructure = [], onSave }) => {
  const [chapters, setChapters] = useState<Chapter[]>(initialStructure.length > 0 ? initialStructure : [
    { id: '1', title: 'Introduction to Physical AI', sections: [] }
  ]);

  const contentTypes = ['text', 'images', 'quizzes', 'summaries', 'code_samples', 'videos'];

  const addChapter = () => {
    const newChapter: Chapter = {
      id: `chapter-${Date.now()}`,
      title: `Chapter ${chapters.length + 1}`,
      sections: []
    };
    setChapters([...chapters, newChapter]);
  };

  const updateChapterTitle = (chapterId: string, title: string) => {
    setChapters(chapters.map(chapter =>
      chapter.id === chapterId ? { ...chapter, title } : chapter
    ));
  };

  const deleteChapter = (chapterId: string) => {
    setChapters(chapters.filter(chapter => chapter.id !== chapterId));
  };

  const addSection = (chapterId: string) => {
    const chapter = chapters.find(c => c.id === chapterId);
    if (chapter) {
      const newSection: Section = {
        id: `section-${Date.now()}`,
        title: `Section ${chapter.sections.length + 1}`,
        contentTypes: ['text']
      };
      setChapters(chapters.map(c =>
        c.id === chapterId
          ? { ...c, sections: [...c.sections, newSection] }
          : c
      ));
    }
  };

  const updateSectionTitle = (chapterId: string, sectionId: string, title: string) => {
    setChapters(chapters.map(chapter =>
      chapter.id === chapterId
        ? {
            ...chapter,
            sections: chapter.sections.map(section =>
              section.id === sectionId ? { ...section, title } : section
            )
          }
        : chapter
    ));
  };

  const toggleContentType = (chapterId: string, sectionId: string, contentType: string) => {
    setChapters(chapters.map(chapter =>
      chapter.id === chapterId
        ? {
            ...chapter,
            sections: chapter.sections.map(section =>
              section.id === sectionId
                ? {
                    ...section,
                    contentTypes: section.contentTypes.includes(contentType)
                      ? section.contentTypes.filter(ct => ct !== contentType)
                      : [...section.contentTypes, contentType]
                  }
                : section
            )
          }
        : chapter
    ));
  };

  const deleteSection = (chapterId: string, sectionId: string) => {
    setChapters(chapters.map(chapter =>
      chapter.id === chapterId
        ? {
            ...chapter,
            sections: chapter.sections.filter(section => section.id !== sectionId)
          }
        : chapter
    ));
  };

  return (
    <Box sx={{ maxWidth: 800, mx: 'auto', p: 3 }}>
      <Typography variant="h4" gutterBottom>
        Customize Textbook Structure
      </Typography>

      <Button
        startIcon={<AddIcon />}
        variant="outlined"
        onClick={addChapter}
        sx={{ mb: 2 }}
      >
        Add Chapter
      </Button>

      <List>
        {chapters.map((chapter) => (
          <Accordion key={chapter.id} sx={{ mb: 1 }}>
            <AccordionSummary expandIcon={<ExpandMoreIcon />}>
              <TextField
                value={chapter.title}
                onChange={(e) => updateChapterTitle(chapter.id, e.target.value)}
                variant="standard"
                size="small"
                sx={{ flex: 1 }}
                placeholder="Chapter title"
              />
              <IconButton
                edge="end"
                aria-label="delete"
                onClick={(e) => {
                  e.stopPropagation();
                  deleteChapter(chapter.id);
                }}
              >
                <DeleteIcon />
              </IconButton>
            </AccordionSummary>
            <AccordionDetails>
              <Button
                startIcon={<AddIcon />}
                variant="outlined"
                size="small"
                onClick={() => addSection(chapter.id)}
                sx={{ mb: 2 }}
              >
                Add Section
              </Button>

              <List>
                {chapter.sections.map((section) => (
                  <ListItem key={section.id} sx={{ border: 1, borderColor: 'grey.300', mb: 1, borderRadius: 1 }}>
                    <TextField
                      value={section.title}
                      onChange={(e) => updateSectionTitle(chapter.id, section.id, e.target.value)}
                      variant="standard"
                      size="small"
                      sx={{ flex: 1, mr: 2 }}
                      placeholder="Section title"
                    />
                    <Box sx={{ flex: 1 }}>
                      {contentTypes.map((type) => (
                        <Button
                          key={type}
                          size="small"
                          variant={section.contentTypes.includes(type) ? 'contained' : 'outlined'}
                          onClick={() => toggleContentType(chapter.id, section.id, type)}
                          sx={{ mr: 1, mb: 1 }}
                        >
                          {type}
                        </Button>
                      ))}
                    </Box>
                    <ListItemSecondaryAction>
                      <IconButton
                        edge="end"
                        aria-label="delete"
                        onClick={() => deleteSection(chapter.id, section.id)}
                      >
                        <DeleteIcon />
                      </IconButton>
                    </ListItemSecondaryAction>
                  </ListItem>
                ))}
              </List>
            </AccordionDetails>
          </Accordion>
        ))}
      </List>

      <Button
        variant="contained"
        onClick={() => onSave(chapters)}
        sx={{ mt: 2 }}
        fullWidth
      >
        Save Structure
      </Button>
    </Box>
  );
};

export default StructureCustomizer;