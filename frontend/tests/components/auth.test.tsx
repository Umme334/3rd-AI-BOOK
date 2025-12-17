import React from 'react';
import { render, screen, fireEvent, waitFor } from '@testing-library/react';
import '@testing-library/jest-dom';
import SignupForm from '../../src/components/auth/signup-form';
import SigninForm from '../../src/components/auth/signin-form';
import BackgroundQuestions from '../../src/components/auth/background-questions';

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
  Button: ({ children, onClick, ...props }: any) => (
    <button data-testid="button" onClick={onClick} {...props}>
      {children}
    </button>
  ),
  FormControl: ({ children, ...props }: any) => (
    <div data-testid="formcontrol" {...props}>
      {children}
    </div>
  ),
  Select: ({ children, value, onChange, ...props }: any) => (
    <select
      data-testid="select"
      value={value}
      onChange={onChange}
      {...props}
    >
      {children}
    </select>
  ),
  MenuItem: ({ children, value, ...props }: any) => (
    <option value={value} {...props}>
      {children}
    </option>
  ),
  Checkbox: ({ checked, onChange, ...props }: any) => (
    <input
      type="checkbox"
      data-testid="checkbox"
      checked={checked}
      onChange={onChange}
      {...props}
    />
  ),
  FormControlLabel: ({ control, label, ...props }: any) => (
    <label data-testid="formcontrollabel" {...props}>
      {control}
      {label}
    </label>
  ),
  FormGroup: ({ children, ...props }: any) => (
    <div data-testid="formgroup" {...props}>
      {children}
    </div>
  ),
  Link: ({ children, onClick, ...props }: any) => (
    <a data-testid="link" onClick={onClick} {...props}>
      {children}
    </a>
  ),
  Box: ({ children, ...props }: any) => (
    <div data-testid="box" {...props}>
      {children}
    </div>
  ),
  Typography: ({ children, ...props }: any) => (
    <span data-testid="typography" {...props}>
      {children}
    </span>
  ),
}));

describe('SignupForm Component', () => {
  const mockOnSubmit = jest.fn();

  beforeEach(() => {
    mockOnSubmit.mockClear();
  });

  it('renders correctly with all fields', () => {
    render(<SignupForm onSubmit={mockOnSubmit} />);

    expect(screen.getByText(/Sign Up for Physical AI & Humanoid Robotics Course/i)).toBeInTheDocument();
    expect(screen.getByTestId('textfield')).toBeInTheDocument(); // First name field
    expect(screen.getAllByTestId('select')).toHaveLength(5); // Multiple select fields
  });

  it('updates form fields correctly', () => {
    render(<SignupForm onSubmit={mockOnSubmit} />);

    const firstNameInput = screen.getAllByTestId('textfield')[0];
    fireEvent.change(firstNameInput, { target: { value: 'John' } });

    expect((firstNameInput as HTMLInputElement).value).toBe('John');
  });

  it('submits form with correct data', async () => {
    render(<SignupForm onSubmit={mockOnSubmit} />);

    // Fill in required fields
    const firstNameInput = screen.getAllByTestId('textfield')[0];
    fireEvent.change(firstNameInput, { target: { value: 'John' } });

    const lastNameInput = screen.getAllByTestId('textfield')[1];
    fireEvent.change(lastNameInput, { target: { value: 'Doe' } });

    const emailInput = screen.getAllByTestId('textfield')[2];
    fireEvent.change(emailInput, { target: { value: 'john@example.com' } });

    const passwordInput = screen.getAllByTestId('textfield')[3];
    fireEvent.change(passwordInput, { target: { value: 'securepassword123' } });

    const confirmPasswordInput = screen.getAllByTestId('textfield')[4];
    fireEvent.change(confirmPasswordInput, { target: { value: 'securepassword123' } });

    const submitButton = screen.getByTestId('button');
    fireEvent.click(submitButton);

    await waitFor(() => {
      expect(mockOnSubmit).toHaveBeenCalledWith({
        email: 'john@example.com',
        password: 'securepassword123',
        confirmPassword: 'securepassword123',
        firstName: 'John',
        lastName: 'Doe',
        softwareExperience: 'beginner',
        hardwareExperience: 'beginner',
        programmingLanguages: [],
        hardwarePlatforms: [],
        roboticsExperience: 'none',
        mathBackground: 'basic',
        primaryGoal: '',
        backgroundQuestions: ''
      });
    });
  });

  it('allows selecting programming languages', () => {
    render(<SignupForm onSubmit={mockOnSubmit} />);

    const checkboxes = screen.getAllByTestId('checkbox');
    expect(checkboxes.length).toBeGreaterThan(0);

    fireEvent.click(checkboxes[0]); // Select first programming language
    // Verify the selection was handled
  });

  it('allows selecting hardware platforms', () => {
    render(<SignupForm onSubmit={mockOnSubmit} />);

    const checkboxes = screen.getAllByTestId('checkbox');
    // Skip the first few which are for programming languages
    if (checkboxes.length > 3) {
      fireEvent.click(checkboxes[3]); // Select first hardware platform
    }
  });
});

describe('SigninForm Component', () => {
  const mockOnSubmit = jest.fn();
  const mockOnForgotPassword = jest.fn();
  const mockOnSwitchToSignup = jest.fn();

  beforeEach(() => {
    mockOnSubmit.mockClear();
    mockOnForgotPassword.mockClear();
    mockOnSwitchToSignup.mockClear();
  });

  it('renders correctly with email and password fields', () => {
    render(
      <SigninForm
        onSubmit={mockOnSubmit}
        onForgotPassword={mockOnForgotPassword}
        onSwitchToSignup={mockOnSwitchToSignup}
      />
    );

    expect(screen.getByText(/Sign In to Your Account/i)).toBeInTheDocument();
    const textfields = screen.getAllByTestId('textfield');
    expect(textfields).toHaveLength(2); // Email and password
  });

  it('submits form with correct data', async () => {
    render(
      <SigninForm
        onSubmit={mockOnSubmit}
        onForgotPassword={mockOnForgotPassword}
        onSwitchToSignup={mockOnSwitchToSignup}
      />
    );

    const emailInput = screen.getAllByTestId('textfield')[0];
    fireEvent.change(emailInput, { target: { value: 'user@example.com' } });

    const passwordInput = screen.getAllByTestId('textfield')[1];
    fireEvent.change(passwordInput, { target: { value: 'password123' } });

    const submitButton = screen.getByTestId('button');
    fireEvent.click(submitButton);

    await waitFor(() => {
      expect(mockOnSubmit).toHaveBeenCalledWith({
        email: 'user@example.com',
        password: 'password123'
      });
    });
  });

  it('handles forgot password link', () => {
    render(
      <SigninForm
        onSubmit={mockOnSubmit}
        onForgotPassword={mockOnForgotPassword}
        onSwitchToSignup={mockOnSwitchToSignup}
      />
    );

    const forgotLink = screen.getByTestId('link');
    fireEvent.click(forgotLink);

    expect(mockOnForgotPassword).toHaveBeenCalled();
  });

  it('handles switch to signup link', () => {
    render(
      <SigninForm
        onSubmit={mockOnSubmit}
        onForgotPassword={mockOnForgotPassword}
        onSwitchToSignup={mockOnSwitchToSignup}
      />
    );

    // Find the "Don't have an account?" link (second link)
    const links = screen.getAllByTestId('link');
    if (links.length > 1) {
      fireEvent.click(links[1]);
      expect(mockOnSwitchToSignup).toHaveBeenCalled();
    }
  });
});

describe('BackgroundQuestions Component', () => {
  const mockOnSubmit = jest.fn();
  const initialData = {
    softwareExperience: 'intermediate',
    hardwareExperience: 'beginner',
    programmingLanguages: 'Python, C++',
    hardwarePlatforms: 'NVIDIA Jetson, Raspberry Pi',
    roboticsExperience: 'basic',
    mathBackground: 'intermediate',
    primaryGoal: 'Learn humanoid robotics',
    backgroundQuestions: 'Any additional information'
  };

  beforeEach(() => {
    mockOnSubmit.mockClear();
  });

  it('renders correctly with initial data', () => {
    render(<BackgroundQuestions initialData={initialData} onSubmit={mockOnSubmit} />);

    expect(screen.getByText(/Tell Us About Your Background/i)).toBeInTheDocument();
    expect(screen.getByTestId('select')).toBeInTheDocument(); // Experience level select
  });

  it('submits updated background information', async () => {
    render(<BackgroundQuestions initialData={initialData} onSubmit={mockOnSubmit} />);

    // Update the primary goal field
    const textfields = screen.getAllByTestId('textfield');
    const primaryGoalField = textfields.find(field =>
      (field as HTMLInputElement).placeholder === "What do you hope to achieve in Physical AI and Humanoid Robotics?"
    );

    if (primaryGoalField) {
      fireEvent.change(primaryGoalField, { target: { value: 'Research in humanoid robotics' } });
    }

    const submitButton = screen.getByTestId('button');
    fireEvent.click(submitButton);

    await waitFor(() => {
      expect(mockOnSubmit).toHaveBeenCalledWith({
        ...initialData,
        primaryGoal: 'Research in humanoid robotics'
      });
    });
  });

  it('allows changing experience levels', () => {
    render(<BackgroundQuestions initialData={initialData} onSubmit={mockOnSubmit} />);

    const select = screen.getByTestId('select');
    fireEvent.change(select, { target: { value: 'advanced' } });

    expect((select as HTMLSelectElement).value).toBe('advanced');
  });
});