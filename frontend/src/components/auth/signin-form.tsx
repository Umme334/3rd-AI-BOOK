import React, { useState } from 'react';
import { Box, TextField, Button, Typography, Link } from '@mui/material';

interface SigninFormData {
  email: string;
  password: string;
}

interface SigninFormProps {
  onSubmit: (data: SigninFormData) => void;
  onForgotPassword?: () => void;
  onSwitchToSignup?: () => void;
}

const SigninForm: React.FC<SigninFormProps> = ({ onSubmit, onForgotPassword, onSwitchToSignup }) => {
  const [formData, setFormData] = useState<SigninFormData>({
    email: '',
    password: ''
  });

  const handleInputChange = (e: React.ChangeEvent<HTMLInputElement>) => {
    const { name, value } = e.target;
    setFormData(prev => ({ ...prev, [name]: value }));
  };

  const handleSubmit = (e: React.FormEvent) => {
    e.preventDefault();
    onSubmit(formData);
  };

  return (
    <Box component="form" onSubmit={handleSubmit} sx={{ maxWidth: 400, mx: 'auto', p: 3 }}>
      <Typography variant="h4" gutterBottom>
        Sign In to Your Account
      </Typography>

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
        sx={{ mt: 2 }}
      />

      {onForgotPassword && (
        <Box sx={{ textAlign: 'right', mt: 1 }}>
          <Link href="#" onClick={(e) => {
            e.preventDefault();
            onForgotPassword();
          }} variant="body2">
            Forgot password?
          </Link>
        </Box>
      )}

      <Button
        type="submit"
        variant="contained"
        size="large"
        sx={{ mt: 3 }}
        fullWidth
      >
        Sign In
      </Button>

      {onSwitchToSignup && (
        <Box sx={{ textAlign: 'center', mt: 2 }}>
          <Typography variant="body2">
            Don't have an account?{' '}
            <Link href="#" onClick={(e) => {
              e.preventDefault();
              onSwitchToSignup();
            }} variant="body2">
              Sign up here
            </Link>
          </Typography>
        </Box>
      )}
    </Box>
  );
};

export default SigninForm;