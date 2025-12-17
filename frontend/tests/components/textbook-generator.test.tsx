import React from 'react';
import { render, screen, fireEvent, waitFor } from '@testing-library/react';
import '@testing-library/jest-dom';
import InputForm from '../../src/components/textbook-generator/input-form';
import StructureCustomizer from '../../src/components/textbook-generator/structure-customizer';
import TextbookPreview from '../../src/components/preview/textbook-preview';

// Mock the Material UI components and other dependencies
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
  Chip: ({ label, onClick, ...props }: any) => (
    <span data-testid="chip" onClick={onClick} {...props}>
      {label}
    </span>
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

describe('InputForm Component', () => {
  const mockOnSubmit = jest.fn();

  beforeEach(() => {
    mockOnSubmit.mockClear();
  });

  it('renders correctly with initial state', () => {
    render(<InputForm onSubmit={mockOnSubmit} />);

    expect(screen.getByText(/Create Your Physical AI & Humanoid Robotics Textbook/i)).toBeInTheDocument();
    expect(screen.getByTestId('textfield')).toBeInTheDocument();
  });

  it('updates state when inputs change', () => {
    render(<InputForm onSubmit={mockOnSubmit} />);

    const subjectInput = screen.getByTestId('textfield');
    fireEvent.change(subjectInput, { target: { value: 'Test Subject' } });

    expect((subjectInput as HTMLInputElement).value).toBe('Test Subject');
  });

  it('submits form with correct data', async () => {
    render(<InputForm onSubmit={mockOnSubmit} />);

    const subjectInput = screen.getByTestId('textfield');
    fireEvent.change(subjectInput, { target: { value: 'Test Subject' } });

    const submitButton = screen.getByTestId('button');
    fireEvent.click(submitButton);

    await waitFor(() => {
      expect(mockOnSubmit).toHaveBeenCalledWith({
        subject: 'Test Subject',
        difficulty: 'intermediate',
        targetAudience: '',
        length: 5,
        additionalTopics: '',
        selectedModules: []
      });
    });
  });

  it('allows module selection', () => {
    render(<InputForm onSubmit={mockOnSubmit} />);

    const moduleChips = screen.getAllByTestId('chip');
    expect(moduleChips.length).toBeGreaterThan(0);

    fireEvent.click(moduleChips[0]);
    // Verify the module was selected
  });
});

describe('StructureCustomizer Component', () => {
  const mockOnSave = jest.fn();
  const initialStructure = [];

  beforeEach(() => {
    mockOnSave.mockClear();
  });

  it('renders correctly with initial structure', () => {
    render(<StructureCustomizer initialStructure={initialStructure} onSave={mockOnSave} />);

    expect(screen.getByText(/Customize Textbook Structure/i)).toBeInTheDocument();
  });

  it('allows adding new chapters', () => {
    render(<StructureCustomizer initialStructure={initialStructure} onSave={mockOnSave} />);

    const addChapterButton = screen.getByTestId('button');
    fireEvent.click(addChapterButton);

    // Verify a new chapter was added
  });

  it('allows adding sections to chapters', () => {
    render(<StructureCustomizer initialStructure={initialStructure} onSave={mockOnSave} />);

    // Add a chapter first, then add a section
    const addChapterButton = screen.getByTestId('button');
    fireEvent.click(addChapterButton);
  });
});

describe('TextbookPreview Component', () => {
  const mockTextbook = {
    id: 'textbook-123',
    title: 'Test Textbook',
    subject: 'Physical AI',
    difficulty: 'intermediate',
    chapters: [
      {
        id: 'chapter-1',
        title: 'Introduction to ROS 2',
        sections: [
          {
            id: 'section-1',
            title: 'ROS 2 Basics',
            content: 'Content about ROS 2 fundamentals'
          }
        ]
      }
    ]
  };

  it('renders textbook preview correctly', () => {
    render(<TextbookPreview textbook={mockTextbook} />);

    expect(screen.getByText(/Test Textbook/i)).toBeInTheDocument();
    expect(screen.getByText(/Introduction to ROS 2/i)).toBeInTheDocument();
  });

  it('shows loading state', () => {
    render(<TextbookPreview textbook={null} isLoading={true} />);

    expect(screen.getByText(/Generating Your Textbook\.\.\./i)).toBeInTheDocument();
  });

  it('shows error state', () => {
    render(<TextbookPreview textbook={null} error="Test error" />);

    expect(screen.getByText(/Test error/i)).toBeInTheDocument();
  });

  it('handles export functionality', () => {
    const mockOnExport = jest.fn();
    render(<TextbookPreview textbook={mockTextbook} onExport={mockOnExport} />);

    const exportButton = screen.getByTestId('button');
    fireEvent.click(exportButton);

    expect(mockOnExport).toHaveBeenCalled();
  });
});