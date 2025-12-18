import React from 'react';
import { BrowserRouter as Router, Routes, Route, Navigate } from 'react-router-dom';
import { ThemeProvider, createTheme } from '@mui/material/styles';
import { CssBaseline } from '@mui/material';
import { Container } from '@mui/material';
import { AuthProvider } from './contexts/AuthContext';

// Import pages
import TextbookGenerationPage from './pages/textbook-generation';
import ChatbotPage from './pages/chatbot-page';
import UserDashboard from './pages/user-dashboard';
import LoginPage from './pages/login-page';
import SignupPage from './pages/signup-page';

// Create theme
const theme = createTheme({
  palette: {
    primary: {
      main: '#1976d2',
    },
    secondary: {
      main: '#e57373',
    },
    background: {
      default: '#f5f5f5',
    },
  },
  typography: {
    fontFamily: [
      '-apple-system',
      'BlinkMacSystemFont',
      '"Segoe UI"',
      'Roboto',
      '"Helvetica Neue"',
      'Arial',
      'sans-serif',
      '"Apple Color Emoji"',
      '"Segoe UI Emoji"',
      '"Segoe UI Symbol"',
    ].join(','),
  },
});

function App() {
  return (
    <ThemeProvider theme={theme}>
      <CssBaseline />
      <Router>
        <AuthProvider>
          <Container maxWidth={false} sx={{ maxWidth: '100vw' }}>
            <Routes>
              {/* Public Routes - Direct access without authentication */}
              <Route path="/" element={<Navigate to="/chatbot" replace />} />
              <Route path="/chatbot" element={<ChatbotPage />} />

              {/* Optional: Keep login/signup for those who want to use them */}
              <Route path="/login" element={<LoginPage />} />
              <Route path="/signup" element={<SignupPage />} />

              {/* Other pages can remain if needed, but chatbot is the main focus */}
              <Route path="/textbook-generation" element={<TextbookGenerationPage />} />
              <Route path="/dashboard" element={<UserDashboard />} />
              <Route path="/textbook/:id" element={<TextbookGenerationPage />} />
            </Routes>
          </Container>
        </AuthProvider>
      </Router>
    </ThemeProvider>
  );
}

export default App;