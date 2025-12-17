import React, { useState } from 'react';
import { Box, TextField, FormControl, InputLabel, Select, MenuItem, Typography, Button } from '@mui/material';

interface BackgroundQuestionsData {
  softwareExperience: string;
  hardwareExperience: string;
  programmingLanguages: string;
  hardwarePlatforms: string;
  roboticsExperience: string;
  mathBackground: string;
  primaryGoal: string;
  backgroundQuestions: string;
}

interface BackgroundQuestionsProps {
  initialData?: BackgroundQuestionsData;
  onSubmit: (data: BackgroundQuestionsData) => void;
}

const BackgroundQuestions: React.FC<BackgroundQuestionsProps> = ({ initialData, onSubmit }) => {
  const [formData, setFormData] = useState<BackgroundQuestionsData>(
    initialData || {
      softwareExperience: 'intermediate',
      hardwareExperience: 'intermediate',
      programmingLanguages: '',
      hardwarePlatforms: '',
      roboticsExperience: 'basic',
      mathBackground: 'intermediate',
      primaryGoal: '',
      backgroundQuestions: ''
    }
  );

  const handleInputChange = (e: React.ChangeEvent<HTMLInputElement | HTMLTextAreaElement>) => {
    const { name, value } = e.target;
    setFormData(prev => ({ ...prev, [name]: value }));
  };

  const handleSelectChange = (name: keyof BackgroundQuestionsData, value: string) => {
    setFormData(prev => ({ ...prev, [name]: value }));
  };

  const handleSubmit = (e: React.FormEvent) => {
    e.preventDefault();
    onSubmit(formData);
  };

  return (
    <Box component="form" onSubmit={handleSubmit} sx={{ maxWidth: 600, mx: 'auto', p: 3 }}>
      <Typography variant="h4" gutterBottom>
        Tell Us About Your Background
      </Typography>

      <Typography variant="body1" sx={{ mb: 3, color: 'text.secondary' }}>
        Help us personalize your Physical AI and Humanoid Robotics learning experience
      </Typography>

      <FormControl fullWidth margin="normal">
        <InputLabel>Software Experience Level</InputLabel>
        <Select
          value={formData.softwareExperience}
          label="Software Experience Level"
          onChange={(e) => handleSelectChange('softwareExperience', e.target.value as string)}
        >
          <MenuItem value="beginner">Beginner</MenuItem>
          <MenuItem value="intermediate">Intermediate</MenuItem>
          <MenuItem value="advanced">Advanced</MenuItem>
        </Select>
      </FormControl>

      <FormControl fullWidth margin="normal">
        <InputLabel>Hardware Experience Level</InputLabel>
        <Select
          value={formData.hardwareExperience}
          label="Hardware Experience Level"
          onChange={(e) => handleSelectChange('hardwareExperience', e.target.value as string)}
        >
          <MenuItem value="beginner">Beginner</MenuItem>
          <MenuItem value="intermediate">Intermediate</MenuItem>
          <MenuItem value="advanced">Advanced</MenuItem>
        </Select>
      </FormControl>

      <FormControl fullWidth margin="normal">
        <InputLabel>Math Background</InputLabel>
        <Select
          value={formData.mathBackground}
          label="Math Background"
          onChange={(e) => handleSelectChange('mathBackground', e.target.value as string)}
        >
          <MenuItem value="basic">Basic (Algebra, Trigonometry)</MenuItem>
          <MenuItem value="intermediate">Intermediate (Calculus, Linear Algebra)</MenuItem>
          <MenuItem value="advanced">Advanced (Differential Equations, Statistics)</MenuItem>
        </Select>
      </FormControl>

      <FormControl fullWidth margin="normal">
        <InputLabel>Robotics Experience</InputLabel>
        <Select
          value={formData.roboticsExperience}
          label="Robotics Experience"
          onChange={(e) => handleSelectChange('roboticsExperience', e.target.value as string)}
        >
          <MenuItem value="none">No Experience</MenuItem>
          <MenuItem value="basic">Basic Experience</MenuItem>
          <MenuItem value="intermediate">Intermediate Experience</MenuItem>
          <MenuItem value="advanced">Advanced Experience</MenuItem>
        </Select>
      </FormControl>

      <TextField
        fullWidth
        label="Programming Languages"
        name="programmingLanguages"
        value={formData.programmingLanguages}
        onChange={handleInputChange}
        margin="normal"
        placeholder="e.g., Python, C++, ROS, MATLAB..."
      />

      <TextField
        fullWidth
        label="Hardware Platforms"
        name="hardwarePlatforms"
        value={formData.hardwarePlatforms}
        onChange={handleInputChange}
        margin="normal"
        placeholder="e.g., NVIDIA Jetson, Raspberry Pi, Arduino..."
      />

      <TextField
        fullWidth
        label="Primary Goal"
        name="primaryGoal"
        value={formData.primaryGoal}
        onChange={handleInputChange}
        margin="normal"
        placeholder="What do you hope to achieve in Physical AI and Humanoid Robotics?"
        multiline
        rows={2}
      />

      <TextField
        fullWidth
        label="Additional Background Information"
        name="backgroundQuestions"
        value={formData.backgroundQuestions}
        onChange={handleInputChange}
        margin="normal"
        placeholder="Any other relevant background information?"
        multiline
        rows={3}
      />

      <Button
        type="submit"
        variant="contained"
        size="large"
        sx={{ mt: 3 }}
        fullWidth
      >
        Save Background Information
      </Button>
    </Box>
  );
};

export default BackgroundQuestions;