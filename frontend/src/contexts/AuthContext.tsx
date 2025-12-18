import React, { createContext, useContext, useState, useEffect, ReactNode } from 'react';
import { useNavigate } from 'react-router-dom';
import { User } from '../types/user';
import apiClient from '../services/api-client';

interface AuthContextType {
  user: User | null;
  isAuthenticated: boolean;
  isLoading: boolean;
  login: (email: string, password: string) => Promise<void>;
  signup: (userData: any) => Promise<void>;
  logout: () => Promise<void>;
  updateProfile: (profileData: Partial<User>) => Promise<void>;
}

const AuthContext = createContext<AuthContextType | undefined>(undefined);

interface AuthProviderProps {
  children: ReactNode;
}

export const AuthProvider: React.FC<AuthProviderProps> = ({ children }) => {
  const [user, setUser] = useState<User | null>(null);
  const [isAuthenticated, setIsAuthenticated] = useState<boolean>(false);
  const [isLoading, setIsLoading] = useState<boolean>(true);
  const navigate = useNavigate();

  // Check if user is authenticated on initial load
  useEffect(() => {
    const initAuth = async () => {
      try {
        // Restore auth token from localStorage if it exists
        const storedToken = localStorage.getItem('better-auth-session');
        if (storedToken) {
          apiClient.setAuthToken(storedToken);
        }

        await checkAuthStatus();
      } catch (error) {
        console.error('Error during auth initialization:', error);
        // Set loading to false even if there's an error
        setIsLoading(false);
      }
    };

    initAuth();
  }, []);

  const checkAuthStatus = async () => {
    try {
      // Try to get user session from backend API
      const response = await fetch('http://localhost:8000/auth/better-auth/session', {
        headers: {
          'Content-Type': 'application/json',
          'Authorization': `Bearer ${localStorage.getItem('better-auth-session') || ''}`
        }
      });

      if (response.ok) {
        const sessionData = await response.json();
        if (sessionData.valid && sessionData.user) {
          // Get full user profile from our API
          const userProfile = await apiClient.getUserProfile(sessionData.user.id);
          setUser(userProfile);
          setIsAuthenticated(true);
        }
      }
    } catch (error) {
      console.error('Error checking auth status:', error);
      // Silently fail, user remains unauthenticated
    } finally {
      setIsLoading(false);
    }
  };

  const login = async (email: string, password: string) => {
    try {
      const response = await apiClient.signin({ email, password });

      // Store the authentication token from the login response
      if (response.access_token) {
        apiClient.setAuthToken(response.access_token);
        // Also store in localStorage for persistence across page reloads
        localStorage.setItem('better-auth-session', response.access_token);
      }

      // Set authentication state
      setUser(response.user_profile);
      setIsAuthenticated(true);

      // Redirect back to the textbook site where the AI assistant is integrated
      window.location.href = 'http://localhost:3005/3rd-AI-BOOK/';
    } catch (error) {
      console.error('Login error:', error);
      throw error;
    }
  };

  const signup = async (userData: any) => {
    try {
      console.log('Starting signup process...', userData);
      const response = await apiClient.signup(userData);
      console.log('Signup API response:', response);

      // Store the authentication token from the signup response
      if (response.access_token) {
        apiClient.setAuthToken(response.access_token);
        // Also store in localStorage for persistence across page reloads
        localStorage.setItem('better-auth-session', response.access_token);
      }

      // After successful signup, set a minimal user state immediately
      // to indicate that the user is authenticated
      if (response.user_id) {
        // Set a minimal user object with the available data
        setUser({
          user_id: response.user_id,
          email: response.email
        });
        setIsAuthenticated(true);

        // Optionally try to get the full profile in the background
        try {
          const profileResponse = await apiClient.getUserProfile(response.user_id);
          console.log('User profile response:', profileResponse);
          // Update the user state with the full profile
          setUser(profileResponse);
        } catch (profileError) {
          console.error('Error getting user profile after signup (will continue anyway):', profileError);
          // Continue with minimal user data if profile fetch fails
        }
      } else {
        throw new Error('Signup response missing user_id');
      }

      console.log('Navigating back to textbook site...');
      // Redirect back to the textbook site where the AI assistant is integrated
      window.location.href = 'http://localhost:3005/3rd-AI-BOOK/';
    } catch (error) {
      console.error('Signup error:', error);
      throw error;
    }
  };

  const logout = async () => {
    try {
      // Call the backend signout endpoint
      const response = await fetch('http://localhost:8000/auth/signout', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          'Authorization': `Bearer ${localStorage.getItem('better-auth-session') || ''}`
        }
      });

      // Even if the backend call fails, we still want to clear local state
      if (!response.ok) {
        console.error('Backend signout failed, but proceeding with local logout');
      }
    } catch (error) {
      console.error('Error calling signout endpoint:', error);
      // Continue with local logout even if backend call fails
    }

    // Clear user data
    setUser(null);
    setIsAuthenticated(false);

    // Clear the local session
    localStorage.removeItem('better-auth-session');

    // Clear the auth token from the API client
    apiClient.clearAuthToken();

    // Redirect to login
    navigate('/login');
  };

  const updateProfile = async (profileData: Partial<User>) => {
    if (!user) {
      throw new Error('User not authenticated');
    }

    try {
      const updatedUser = await apiClient.updateUserProfile(user.id, profileData);
      setUser(updatedUser);
      return updatedUser;
    } catch (error) {
      console.error('Update profile error:', error);
      throw error;
    }
  };

  const value = {
    user,
    isAuthenticated,
    isLoading,
    login,
    signup,
    logout,
    updateProfile,
  };

  return (
    <AuthContext.Provider value={value}>
      {children}
    </AuthContext.Provider>
  );
};

export const useAuth = () => {
  const context = useContext(AuthContext);
  if (context === undefined) {
    throw new Error('useAuth must be used within an AuthProvider');
  }
  return context;
};