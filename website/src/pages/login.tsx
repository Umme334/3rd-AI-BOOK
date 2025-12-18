import React, { useEffect } from 'react';
import Layout from '@theme/Layout';

const LoginPage = () => {
  useEffect(() => {
    // Redirect to the frontend login page
    // If the frontend is running on localhost:3004, redirect there
    // In production, this would redirect to the deployed frontend
    if (typeof window !== 'undefined') {
      // Check if we're in development or production
      const isDevelopment = window.location.hostname === 'localhost' || window.location.hostname === '127.0.0.1';

      if (isDevelopment) {
        // In development, redirect to the frontend login page
        window.location.href = 'http://localhost:3006/login';
      } else {
        // In production, redirect to the deployed frontend login page
        // This should be updated to the actual deployed URL
        window.location.href = 'http://localhost:3006/login'; // Placeholder - update in production
      }
    }
  }, []);

  return (
    <Layout title="Redirecting to Login" description="Redirecting to login page">
      <div style={{
        display: 'flex',
        justifyContent: 'center',
        alignItems: 'center',
        height: '100vh',
        flexDirection: 'column'
      }}>
        <h1>Redirecting to Login</h1>
        <p>If you are not redirected automatically, <a href="http://localhost:3004/login">click here</a> to go to the login page.</p>
        <div className="loader" style={{ marginTop: '20px' }}>Redirecting...</div>
      </div>
    </Layout>
  );
};

export default LoginPage;