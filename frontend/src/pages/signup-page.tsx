import React from 'react';
import { Container, Box, Typography } from '@mui/material';
import { useAuth } from '../contexts/AuthContext';
import SignupForm from '../components/auth/signup-form';
import { useNavigate } from 'react-router-dom';

const SignupPage: React.FC = () => {
  const { signup } = useAuth();
  const navigate = useNavigate();

  const handleSignup = async (data: any) => {
    try {
      // Transform the signup data to match our API
      const signupData = {
        email: data.email,
        password: data.password,
        firstName: data.firstName,
        lastName: data.lastName || '',
        softwareExperience: data.softwareExperience,
        hardwareExperience: data.hardwareExperience,
        programmingLanguages: data.programmingLanguages || [],
        hardwarePlatforms: data.hardwarePlatforms || [],
        roboticsExperience: data.roboticsExperience,
        mathBackground: data.mathBackground,
        primaryGoal: data.primaryGoal,
        backgroundQuestions: data.backgroundQuestions || ''
      };

      await signup(signupData);
    } catch (error) {
      console.error('Signup failed:', error);
      // Handle signup error (show notification, etc.)
    }
  };

  const handleSwitchToLogin = () => {
    navigate('/login');
  };

  return (
    <Container maxWidth="sm" sx={{ py: 4 }}>
      <Box sx={{
        display: 'flex',
        flexDirection: 'column',
        alignItems: 'center',
        justifyContent: 'center',
        minHeight: '80vh'
      }}>
        <Box sx={{ width: '100%', maxWidth: 600 }}>
          <Typography variant="h4" align="center" gutterBottom>
            Join Physical AI & Humanoid Robotics
          </Typography>
          <Typography variant="body1" align="center" color="text.secondary" sx={{ mb: 3 }}>
            Create an account to personalize your learning experience
          </Typography>

          <SignupForm
            onSubmit={handleSignup}
          />

          <Box sx={{ textAlign: 'center', mt: 2 }}>
            <Typography variant="body2">
              Already have an account?{' '}
              <span
                style={{ color: '#1976d2', cursor: 'pointer', textDecoration: 'underline' }}
                onClick={handleSwitchToLogin}
              >
                Sign in here
              </span>
            </Typography>
          </Box>
        </Box>
      </Box>
    </Container>
  );
};

export default SignupPage;