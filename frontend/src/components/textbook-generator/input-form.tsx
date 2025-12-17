import React, { useState } from 'react';
import { TextField, Button, Box, Typography, FormControl, InputLabel, Select, MenuItem, Chip, Stack } from '@mui/material';

interface InputFormProps {
  onSubmit: (data: any) => void;
}

const InputForm: React.FC<InputFormProps> = ({ onSubmit }) => {
  const [subject, setSubject] = useState('');
  const [difficulty, setDifficulty] = useState('intermediate');
  const [targetAudience, setTargetAudience] = useState('');
  const [length, setLength] = useState(5);
  const [additionalTopics, setAdditionalTopics] = useState('');
  const [selectedModules, setSelectedModules] = useState<string[]>([]);

  const modules = [
    'ROS 2',
    'Gazebo',
    'NVIDIA Isaac',
    'Vision-Language-Action',
    'Control Systems',
    'Perception',
    'Planning',
    'Learning'
  ];

  const handleSubmit = (e: React.FormEvent) => {
    e.preventDefault();
    onSubmit({
      subject,
      difficulty,
      targetAudience,
      length,
      additionalTopics,
      selectedModules
    });
  };

  const handleModuleToggle = (module: string) => {
    if (selectedModules.includes(module)) {
      setSelectedModules(selectedModules.filter(m => m !== module));
    } else {
      setSelectedModules([...selectedModules, module]);
    }
  };

  return (
    <Box component="form" onSubmit={handleSubmit} sx={{ maxWidth: 600, mx: 'auto', p: 3 }}>
      <Typography variant="h4" gutterBottom>
        Create Your Physical AI & Humanoid Robotics Textbook
      </Typography>

      <TextField
        fullWidth
        label="Subject/Topic"
        value={subject}
        onChange={(e) => setSubject(e.target.value)}
        margin="normal"
        required
        placeholder="e.g., Introduction to Humanoid Robotics, Bipedal Locomotion, etc."
      />

      <TextField
        fullWidth
        label="Target Audience"
        value={targetAudience}
        onChange={(e) => setTargetAudience(e.target.value)}
        margin="normal"
        placeholder="e.g., Undergraduate students, Graduate students, Researchers"
      />

      <FormControl fullWidth margin="normal">
        <InputLabel>Difficulty Level</InputLabel>
        <Select
          value={difficulty}
          label="Difficulty Level"
          onChange={(e) => setDifficulty(e.target.value)}
        >
          <MenuItem value="beginner">Beginner</MenuItem>
          <MenuItem value="intermediate">Intermediate</MenuItem>
          <MenuItem value="advanced">Advanced</MenuItem>
        </Select>
      </FormControl>

      <TextField
        fullWidth
        label="Number of Chapters"
        type="number"
        value={length}
        onChange={(e) => setLength(parseInt(e.target.value) || 5)}
        margin="normal"
        inputProps={{ min: 1, max: 20 }}
      />

      <TextField
        fullWidth
        label="Additional Topics (comma separated)"
        value={additionalTopics}
        onChange={(e) => setAdditionalTopics(e.target.value)}
        margin="normal"
        placeholder="e.g., ROS 2, Gazebo, NVIDIA Isaac, Vision-Language-Action"
      />

      <Box sx={{ mt: 2, mb: 2 }}>
        <Typography variant="h6" gutterBottom>
          Select Modules to Include:
        </Typography>
        <Stack direction="row" spacing={1} flexWrap="wrap" sx={{ gap: 1 }}>
          {modules.map((module) => (
            <Chip
              key={module}
              label={module}
              onClick={() => handleModuleToggle(module)}
              color={selectedModules.includes(module) ? 'primary' : 'default'}
              variant={selectedModules.includes(module) ? 'filled' : 'outlined'}
            />
          ))}
        </Stack>
      </Box>

      <Button
        type="submit"
        variant="contained"
        size="large"
        sx={{ mt: 3 }}
        fullWidth
      >
        Generate Textbook
      </Button>
    </Box>
  );
};

export default InputForm;