import React from 'react';
import { Container, Box, Typography } from '@mui/material';
import { useAuth } from '../contexts/AuthContext';
import SigninForm from '../components/auth/signin-form';
import { useNavigate } from 'react-router-dom';

const LoginPage: React.FC = () => {
  const { login } = useAuth();
  const navigate = useNavigate();

  const handleLogin = async (data: { email: string; password: string }) => {
    try {
      await login(data.email, data.password);
    } catch (error) {
      console.error('Login failed:', error);
      // Handle login error (show notification, etc.)
    }
  };

  const handleSwitchToSignup = () => {
    navigate('/signup');
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
        <Box sx={{ width: '100%', maxWidth: 400 }}>
          <Typography variant="h4" align="center" gutterBottom>
            Welcome Back
          </Typography>
          <Typography variant="body1" align="center" color="text.secondary" sx={{ mb: 3 }}>
            Sign in to your Physical AI & Humanoid Robotics account
          </Typography>

          <SigninForm
            onSubmit={handleLogin}
            onSwitchToSignup={handleSwitchToSignup}
          />
        </Box>
      </Box>
    </Container>
  );
};

export default LoginPage;