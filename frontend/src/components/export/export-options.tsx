import React, { useState } from 'react';
import { Box, Button, Typography, FormControl, InputLabel, Select, MenuItem, LinearProgress, Alert } from '@mui/material';

interface ExportOptionsProps {
  textbookId: string;
  onExportComplete?: (format: string, url: string) => void;
}

const ExportOptions: React.FC<ExportOptionsProps> = ({ textbookId, onExportComplete }) => {
  const [exportFormat, setExportFormat] = useState('docusaurus');
  const [isExporting, setIsExporting] = useState(false);
  const [exportError, setExportError] = useState<string | null>(null);

  const exportFormats = [
    { value: 'docusaurus', label: 'Docusaurus (GitHub Pages)' },
    { value: 'pdf', label: 'PDF' },
    { value: 'html', label: 'HTML' },
    { value: 'markdown', label: 'Markdown' }
  ];

  const handleExport = async () => {
    setIsExporting(true);
    setExportError(null);

    try {
      // Simulate API call to backend
      await new Promise(resolve => setTimeout(resolve, 2000)); // Simulate network delay

      // In a real implementation, this would call the backend API
      // const response = await fetch(`/api/textbooks/${textbookId}/export`, {
      //   method: 'POST',
      //   headers: { 'Content-Type': 'application/json' },
      //   body: JSON.stringify({ format: exportFormat })
      // });

      // const result = await response.json();

      // Simulate success response
      const mockUrl = `/exports/textbook-${textbookId}.${exportFormat}`;

      if (onExportComplete) {
        onExportComplete(exportFormat, mockUrl);
      }
    } catch (error) {
      setExportError('Failed to export textbook. Please try again.');
      console.error('Export error:', error);
    } finally {
      setIsExporting(false);
    }
  };

  return (
    <Box sx={{ p: 3, maxWidth: 500, mx: 'auto' }}>
      <Typography variant="h5" gutterBottom>
        Export Your Textbook
      </Typography>

      <FormControl fullWidth margin="normal">
        <InputLabel>Export Format</InputLabel>
        <Select
          value={exportFormat}
          label="Export Format"
          onChange={(e) => setExportFormat(e.target.value)}
        >
          {exportFormats.map((format) => (
            <MenuItem key={format.value} value={format.value}>
              {format.label}
            </MenuItem>
          ))}
        </Select>
      </FormControl>

      {isExporting && (
        <Box sx={{ mt: 2 }}>
          <LinearProgress />
          <Typography variant="body2" sx={{ mt: 1 }}>
            Preparing your {exportFormat} export...
          </Typography>
        </Box>
      )}

      {exportError && (
        <Alert severity="error" sx={{ mt: 2 }}>
          {exportError}
        </Alert>
      )}

      <Button
        variant="contained"
        onClick={handleExport}
        disabled={isExporting}
        sx={{ mt: 2 }}
        fullWidth
      >
        {isExporting ? 'Exporting...' : 'Export Textbook'}
      </Button>

      <Box sx={{ mt: 3 }}>
        <Typography variant="h6" gutterBottom>
          Export Options:
        </Typography>
        <ul>
          <li><strong>Docusaurus:</strong> Perfect for GitHub Pages deployment</li>
          <li><strong>PDF:</strong> Print-ready format with all content and images</li>
          <li><strong>HTML:</strong> Web-friendly format with interactive elements</li>
          <li><strong>Markdown:</strong> Simple format for further editing</li>
        </ul>
      </Box>
    </Box>
  );
};

export default ExportOptions;