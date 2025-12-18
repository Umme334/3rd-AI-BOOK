import React, { useState } from 'react';
import { Box, Typography, Paper, Container, Accordion, AccordionSummary, AccordionDetails, List, ListItem, ListItemText } from '@mui/material';
import ExpandMoreIcon from '@mui/icons-material/ExpandMore';

interface Chapter {
  id: string;
  title: string;
  sections: Section[];
}

interface Section {
  id: string;
  title: string;
  content: string;
}

interface Textbook {
  id: string;
  title: string;
  subject: string;
  description: string;
  chapters: Chapter[];
}

interface TextbookViewerProps {
  textbook: Textbook;
}

const TextbookViewer: React.FC<TextbookViewerProps> = ({ textbook }) => {
  const [expandedChapter, setExpandedChapter] = useState<string | false>(false);

  const handleChapterChange = (chapterId: string) => (event: React.SyntheticEvent, isExpanded: boolean) => {
    setExpandedChapter(isExpanded ? chapterId : false);
  };

  return (
    <Container maxWidth="md" sx={{ py: 3 }}>
      <Paper sx={{ p: 3, mb: 3 }}>
        <Typography variant="h3" component="h1" gutterBottom>
          {textbook.title}
        </Typography>
        <Typography variant="h6" color="text.secondary" gutterBottom>
          {textbook.subject}
        </Typography>
        <Typography variant="body1" paragraph>
          {textbook.description}
        </Typography>
      </Paper>

      <Paper sx={{ p: 3 }}>
        <Typography variant="h4" component="h2" gutterBottom>
          Table of Contents
        </Typography>
        <List>
          {textbook.chapters.map((chapter, index) => (
            <Accordion
              key={chapter.id}
              expanded={expandedChapter === chapter.id}
              onChange={handleChapterChange(chapter.id)}
              sx={{ mb: 1 }}
            >
              <AccordionSummary
                expandIcon={<ExpandMoreIcon />}
                sx={{
                  backgroundColor: expandedChapter === chapter.id ? 'action.selected' : 'inherit',
                  '&:hover': { backgroundColor: 'action.hover' }
                }}
              >
                <Typography variant="h6">
                  Chapter {index + 1}: {chapter.title}
                </Typography>
              </AccordionSummary>
              <AccordionDetails>
                <List>
                  {chapter.sections.map((section, sectionIndex) => (
                    <ListItem key={section.id} sx={{ pl: 2 }}>
                      <ListItemText
                        primary={`Section ${index + 1}.${sectionIndex + 1}: ${section.title}`}
                        secondary={section.content.substring(0, 100) + (section.content.length > 100 ? '...' : '')}
                      />
                    </ListItem>
                  ))}
                </List>
              </AccordionDetails>
            </Accordion>
          ))}
        </List>
      </Paper>
    </Container>
  );
};

export default TextbookViewer;