import React from 'react';
import { render, screen } from '@testing-library/react';
import App from '../src/App';

// Mock the child components to avoid complex dependencies
jest.mock('../src/components/textbook-generator', () => ({
  __esModule: true,
  default: () => <div data-testid="textbook-generator">Textbook Generator</div>,
}));

jest.mock('../src/components/chatbot/chatbot-widget', () => ({
  __esModule: true,
  ChatbotWidget: () => <div data-testid="chatbot-widget">Chatbot Widget</div>,
}));

jest.mock('../src/components/auth/signup-form', () => ({
  __esModule: true,
  default: () => <div data-testid="signup-form">Signup Form</div>,
}));

jest.mock('../src/components/auth/signin-form', () => ({
  __esModule: true,
  default: () => <div data-testid="signin-form">Signin Form</div>,
}));

jest.mock('../src/components/personalization/personalization-toggle', () => ({
  __esModule: true,
  default: () => <div data-testid="personalization-toggle">Personalization Toggle</div>,
}));

describe('App Component', () => {
  test('renders without crashing', () => {
    render(<App />);
    expect(screen.getByText('AI-Native Textbook for Physical AI and Humanoid Robotics')).toBeInTheDocument();
  });

  test('renders navigation elements', () => {
    render(<App />);

    // Check for navigation elements
    expect(screen.getByText('Home')).toBeInTheDocument();
    expect(screen.getByText('Textbook Generator')).toBeInTheDocument();
    expect(screen.getByText('Chatbot')).toBeInTheDocument();
    expect(screen.getByText('Login')).toBeInTheDocument();
  });

  test('renders textbook generator component', () => {
    render(<App />);
    expect(screen.getByTestId('textbook-generator')).toBeInTheDocument();
  });

  test('renders chatbot widget component', () => {
    render(<App />);
    expect(screen.getByTestId('chatbot-widget')).toBeInTheDocument();
  });
});

// Test for TextbookGenerator component
describe('TextbookGenerator Component', () => {
  const TextbookGenerator = require('../src/components/textbook-generator').default;

  test('renders textbook generator form', () => {
    render(<TextbookGenerator />);

    expect(screen.getByLabelText(/Title:/i)).toBeInTheDocument();
    expect(screen.getByLabelText(/Subject:/i)).toBeInTheDocument();
    expect(screen.getByLabelText(/Difficulty:/i)).toBeInTheDocument();
    expect(screen.getByText('Generate Textbook')).toBeInTheDocument();
  });

  test('has proper form fields', () => {
    render(<TextbookGenerator />);

    const titleInput = screen.getByLabelText(/Title:/i);
    const subjectInput = screen.getByLabelText(/Subject:/i);
    const difficultySelect = screen.getByLabelText(/Difficulty:/i);

    expect(titleInput).toBeInTheDocument();
    expect(subjectInput).toBeInTheDocument();
    expect(difficultySelect).toBeInTheDocument();
  });
});

// Test for ChatbotWidget component
describe('ChatbotWidget Component', () => {
  const { ChatbotWidget } = require('../src/components/chatbot/chatbot-widget');

  test('renders chatbot toggle button initially', () => {
    render(<ChatbotWidget textbookId="test-id" />);

    expect(screen.getByText('🤖 AI Assistant')).toBeInTheDocument();
  });
});

// Test for SignupForm component
describe('SignupForm Component', () => {
  const SignupForm = require('../src/components/auth/signup-form').default;

  test('renders signup form', () => {
    render(<SignupForm />);

    expect(screen.getByLabelText(/Email:/i)).toBeInTheDocument();
    expect(screen.getByLabelText(/Password:/i)).toBeInTheDocument();
    expect(screen.getByLabelText(/First Name:/i)).toBeInTheDocument();
    expect(screen.getByLabelText(/Last Name:/i)).toBeInTheDocument();
    expect(screen.getByText('Sign Up')).toBeInTheDocument();
  });
});

// Test for SigninForm component
describe('SigninForm Component', () => {
  const SigninForm = require('../src/components/auth/signin-form').default;

  test('renders signin form', () => {
    render(<SigninForm />);

    expect(screen.getByLabelText(/Email:/i)).toBeInTheDocument();
    expect(screen.getByLabelText(/Password:/i)).toBeInTheDocument();
    expect(screen.getByText('Sign In')).toBeInTheDocument();
  });
});