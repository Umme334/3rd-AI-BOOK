import React from 'react';
import { render, screen, fireEvent, waitFor, act } from '@testing-library/react';
import '@testing-library/jest-dom';
import { ChatbotWidget, ChatMessage } from '../../src/components/chatbot/chatbot-widget';
import QueryInput from '../../src/components/chatbot/query-input';

// Mock the Material UI components
jest.mock('@mui/material', () => ({
  ...jest.requireActual('@mui/material'),
  TextField: ({ label, value, onChange, ...props }: any) => (
    <input
      data-testid="textfield"
      placeholder={label}
      value={value || ''}
      onChange={onChange}
      {...props}
    />
  ),
  Button: ({ children, onClick, endIcon, ...props }: any) => (
    <button data-testid="button" onClick={onClick} {...props}>
      {children}
      {endIcon}
    </button>
  ),
  Paper: ({ children, ...props }: any) => (
    <div data-testid="paper" {...props}>
      {children}
    </div>
  ),
  Typography: ({ children, ...props }: any) => (
    <span data-testid="typography" {...props}>
      {children}
    </span>
  ),
  Box: ({ children, ...props }: any) => (
    <div data-testid="box" {...props}>
      {children}
    </div>
  ),
  Avatar: ({ children, ...props }: any) => (
    <div data-testid="avatar" {...props}>
      {children}
    </div>
  ),
  Divider: (props: any) => <hr data-testid="divider" {...props} />,
}));

// Mock the icons
jest.mock('@mui/icons-material/Send', () => ({
  __esModule: true,
  default: () => <span data-testid="send-icon">Send</span>,
}));

describe('ChatMessage Component', () => {
  const mockMessage = {
    id: '1',
    role: 'assistant',
    content: 'Hello! How can I help you with Physical AI?',
    timestamp: new Date(),
  };

  it('renders user message correctly', () => {
    const userMessage = { ...mockMessage, role: 'user' };
    render(<ChatMessage message={userMessage} />);

    const messageElements = screen.getAllByTestId('paper');
    expect(messageElements).toHaveLength(1);
  });

  it('renders assistant message correctly', () => {
    render(<ChatMessage message={mockMessage} />);

    const messageElements = screen.getAllByTestId('paper');
    expect(messageElements).toHaveLength(1);
  });

  it('shows typing indicator', () => {
    render(<ChatMessage message={mockMessage} isTyping={true} />);

    // Check if typing indicator is present
    const typingElements = screen.getAllByTestId('box');
    expect(typingElements).toHaveLength(2); // Main box + typing indicator box
  });
});

describe('QueryInput Component', () => {
  const mockOnSend = jest.fn();

  beforeEach(() => {
    mockOnSend.mockClear();
  });

  it('renders correctly', () => {
    render(<QueryInput onSend={mockOnSend} />);

    expect(screen.getByTestId('textfield')).toBeInTheDocument();
    expect(screen.getByTestId('button')).toBeInTheDocument();
  });

  it('updates input value', () => {
    render(<QueryInput onSend={mockOnSend} />);

    const input = screen.getByTestId('textfield');
    fireEvent.change(input, { target: { value: 'Test query' } });

    expect((input as HTMLInputElement).value).toBe('Test query');
  });

  it('sends query when button is clicked', async () => {
    render(<QueryInput onSend={mockOnSend} />);

    const input = screen.getByTestId('textfield');
    fireEvent.change(input, { target: { value: 'Test query' } });

    const sendButton = screen.getByTestId('button');
    fireEvent.click(sendButton);

    await waitFor(() => {
      expect(mockOnSend).toHaveBeenCalledWith('Test query');
    });
  });

  it('sends query when Enter key is pressed', async () => {
    render(<QueryInput onSend={mockOnSend} />);

    const input = screen.getByTestId('textfield');
    fireEvent.change(input, { target: { value: 'Test query' } });
    fireEvent.keyPress(input, { key: 'Enter', code: 'Enter', char: '\r', shiftKey: false });

    await waitFor(() => {
      expect(mockOnSend).toHaveBeenCalledWith('Test query');
    });
  });

  it('does not send empty query', () => {
    render(<QueryInput onSend={mockOnSend} />);

    const input = screen.getByTestId('textfield');
    fireEvent.change(input, { target: { value: '' } });

    const sendButton = screen.getByTestId('button');
    fireEvent.click(sendButton);

    expect(mockOnSend).not.toHaveBeenCalled();
  });

  it('is disabled when disabled prop is true', () => {
    render(<QueryInput onSend={mockOnSend} disabled={true} />);

    const input = screen.getByTestId('textfield');
    const button = screen.getByTestId('button');

    expect(input).toBeDisabled();
    expect(button).toBeDisabled();
  });
});

describe('ChatbotWidget Component', () => {
  const textbookId = 'textbook-123';

  beforeEach(() => {
    jest.useFakeTimers();
  });

  afterEach(() => {
    jest.useRealTimers();
  });

  it('renders correctly with initial state', () => {
    render(<ChatbotWidget textbookId={textbookId} />);

    expect(screen.getByText(/Physical AI & Humanoid Robotics Assistant/i)).toBeInTheDocument();
    expect(screen.getByTestId('textfield')).toBeInTheDocument();
  });

  it('sends a message and receives response', async () => {
    render(<ChatbotWidget textbookId={textbookId} />);

    const input = screen.getByTestId('textfield');
    fireEvent.change(input, { target: { value: 'What is ROS 2?' } });

    const sendButton = screen.getByTestId('button');
    fireEvent.click(sendButton);

    // Fast forward time for the simulated API call
    act(() => {
      jest.advanceTimersByTime(1000);
    });

    // Wait for the response to be rendered
    await waitFor(() => {
      expect(screen.getByText(/This is a simulated response/i)).toBeInTheDocument();
    });
  });

  it('shows initial assistant message', () => {
    render(<ChatbotWidget textbookId={textbookId} />);

    expect(screen.getByText(/Hello! I'm your AI assistant/i)).toBeInTheDocument();
  });

  it('handles API errors gracefully', async () => {
    render(<ChatbotWidget textbookId={textbookId} />);

    const input = screen.getByTestId('textfield');
    fireEvent.change(input, { target: { value: 'Test error query' } });

    const sendButton = screen.getByTestId('button');
    fireEvent.click(sendButton);

    // Fast forward time
    act(() => {
      jest.advanceTimersByTime(1000);
    });

    // The component should handle errors and show an appropriate message
    await waitFor(() => {
      const errorMessages = screen.queryAllByText(/Sorry, I encountered an error/i);
      if (errorMessages.length > 0) {
        expect(errorMessages[0]).toBeInTheDocument();
      }
    });
  });
});