import React, { useState } from 'react';
import { Box, TextField, Button, Typography, FormControl, InputLabel, Select, MenuItem, Checkbox, FormControlLabel, FormGroup } from '@mui/material';

interface SignupFormData {
  email: string;
  password: string;
  confirmPassword: string;
  firstName: string;
  lastName: string;
  softwareExperience: string;
  hardwareExperience: string;
  programmingLanguages: string[];
  hardwarePlatforms: string[];
  roboticsExperience: string;
  mathBackground: string;
  primaryGoal: string;
  backgroundQuestions: string;
}

interface SignupFormProps {
  onSubmit: (data: SignupFormData) => void;
}

const SignupForm: React.FC<SignupFormProps> = ({ onSubmit }) => {
  const [formData, setFormData] = useState<SignupFormData>({
    email: '',
    password: '',
    confirmPassword: '',
    firstName: '',
    lastName: '',
    softwareExperience: 'beginner',
    hardwareExperience: 'beginner',
    programmingLanguages: [],
    hardwarePlatforms: [],
    roboticsExperience: 'none',
    mathBackground: 'basic',
    primaryGoal: '',
    backgroundQuestions: ''
  });

  const programmingLanguages = [
    'Python',
    'C++',
    'ROS',
    'MATLAB',
    'Java',
    'JavaScript',
    'C',
    'Other'
  ];

  const hardwarePlatforms = [
    'NVIDIA Jetson',
    'Raspberry Pi',
    'Arduino',
    'ROS Robots',
    'Gazebo',
    'V-REP/CoppeliaSim',
    'Other'
  ];

  const handleInputChange = (e: React.ChangeEvent<HTMLInputElement>) => {
    const { name, value } = e.target;
    setFormData(prev => ({ ...prev, [name]: value }));
  };

  const handleCheckboxChange = (category: 'programmingLanguages' | 'hardwarePlatforms', value: string) => {
    setFormData(prev => {
      const currentValues = prev[category] as string[];
      if (currentValues.includes(value)) {
        return {
          ...prev,
          [category]: currentValues.filter(item => item !== value)
        };
      } else {
        return {
          ...prev,
          [category]: [...currentValues, value]
        };
      }
    });
  };

  const handleSubmit = (e: React.FormEvent) => {
    e.preventDefault();
    onSubmit(formData);
  };

  return (
    <Box component="form" onSubmit={handleSubmit} sx={{ maxWidth: 500, mx: 'auto', p: 3 }}>
      <Typography variant="h4" gutterBottom>
        Sign Up for Physical AI & Humanoid Robotics Course
      </Typography>

      <Typography variant="body1" sx={{ mb: 2 }}>
        Please provide your background information to personalize your learning experience.
      </Typography>

      <TextField
        fullWidth
        label="First Name"
        name="firstName"
        value={formData.firstName}
        onChange={handleInputChange}
        margin="normal"
        required
      />

      <TextField
        fullWidth
        label="Last Name"
        name="lastName"
        value={formData.lastName}
        onChange={handleInputChange}
        margin="normal"
        required
      />

      <TextField
        fullWidth
        label="Email"
        name="email"
        type="email"
        value={formData.email}
        onChange={handleInputChange}
        margin="normal"
        required
      />

      <TextField
        fullWidth
        label="Password"
        name="password"
        type="password"
        value={formData.password}
        onChange={handleInputChange}
        margin="normal"
        required
      />

      <TextField
        fullWidth
        label="Confirm Password"
        name="confirmPassword"
        type="password"
        value={formData.confirmPassword}
        onChange={handleInputChange}
        margin="normal"
        required
      />

      <FormControl fullWidth margin="normal">
        <InputLabel>Software Experience Level</InputLabel>
        <Select
          name="softwareExperience"
          value={formData.softwareExperience}
          label="Software Experience Level"
          onChange={(e) => setFormData(prev => ({ ...prev, softwareExperience: e.target.value as string }))}
        >
          <MenuItem value="beginner">Beginner</MenuItem>
          <MenuItem value="intermediate">Intermediate</MenuItem>
          <MenuItem value="advanced">Advanced</MenuItem>
        </Select>
      </FormControl>

      <FormControl fullWidth margin="normal">
        <InputLabel>Hardware Experience Level</InputLabel>
        <Select
          name="hardwareExperience"
          value={formData.hardwareExperience}
          label="Hardware Experience Level"
          onChange={(e) => setFormData(prev => ({ ...prev, hardwareExperience: e.target.value as string }))}
        >
          <MenuItem value="beginner">Beginner</MenuItem>
          <MenuItem value="intermediate">Intermediate</MenuItem>
          <MenuItem value="advanced">Advanced</MenuItem>
        </Select>
      </FormControl>

      <FormControl fullWidth margin="normal">
        <InputLabel>Math Background</InputLabel>
        <Select
          name="mathBackground"
          value={formData.mathBackground}
          label="Math Background"
          onChange={(e) => setFormData(prev => ({ ...prev, mathBackground: e.target.value as string }))}
        >
          <MenuItem value="basic">Basic (Algebra, Trigonometry)</MenuItem>
          <MenuItem value="intermediate">Intermediate (Calculus, Linear Algebra)</MenuItem>
          <MenuItem value="advanced">Advanced (Differential Equations, Statistics)</MenuItem>
        </Select>
      </FormControl>

      <FormControl fullWidth margin="normal">
        <InputLabel>Robotics Experience</InputLabel>
        <Select
          name="roboticsExperience"
          value={formData.roboticsExperience}
          label="Robotics Experience"
          onChange={(e) => setFormData(prev => ({ ...prev, roboticsExperience: e.target.value as string }))}
        >
          <MenuItem value="none">No Experience</MenuItem>
          <MenuItem value="basic">Basic Experience</MenuItem>
          <MenuItem value="intermediate">Intermediate Experience</MenuItem>
          <MenuItem value="advanced">Advanced Experience</MenuItem>
        </Select>
      </FormControl>

      <Typography variant="h6" sx={{ mt: 2, mb: 1 }}>
        Programming Languages:
      </Typography>
      <FormGroup>
        {programmingLanguages.map(lang => (
          <FormControlLabel
            key={lang}
            control={
              <Checkbox
                checked={formData.programmingLanguages.includes(lang)}
                onChange={() => handleCheckboxChange('programmingLanguages', lang)}
              />
            }
            label={lang}
          />
        ))}
      </FormGroup>

      <Typography variant="h6" sx={{ mt: 2, mb: 1 }}>
        Hardware Platforms:
      </Typography>
      <FormGroup>
        {hardwarePlatforms.map(platform => (
          <FormControlLabel
            key={platform}
            control={
              <Checkbox
                checked={formData.hardwarePlatforms.includes(platform)}
                onChange={() => handleCheckboxChange('hardwarePlatforms', platform)}
              />
            }
            label={platform}
          />
        ))}
      </FormGroup>

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
        label="Background Questions"
        name="backgroundQuestions"
        value={formData.backgroundQuestions}
        onChange={handleInputChange}
        margin="normal"
        placeholder="Any additional background information you'd like to share?"
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
        Sign Up
      </Button>
    </Box>
  );
};

export default SignupForm;