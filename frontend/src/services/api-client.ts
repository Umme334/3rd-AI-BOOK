import {
  TextbookCreateRequest,
  TextbookResponse,
  ProgressResponse,
  ExportRequest
} from '../types/textbook';

interface ApiResponse<T = any> {
  data: T;
  message?: string;
  success: boolean;
  error?: string;
}

interface ChatbotQueryRequest {
  textbook_id: string;
  query: string;
  session_id?: string;
}

interface ChatbotSessionCreateRequest {
  textbook_id: string;
  user_id?: string;
  session_name?: string;
}

interface SignupRequest {
  email: string;
  password: string;
  firstName: string;
  lastName: string;
  softwareExperience: string;
  hardwareExperience: string;
  programmingLanguages: string[];
  hardwarePlatforms: string[];
  roboticsExperience: string;
  mathBackground: string;
  primaryGoal: string;
  backgroundQuestions: string;
}

interface SigninRequest {
  email: string;
  password: string;
}

interface PersonalizationRequest {
  content: string;
  user_id: string;
  content_type: string;
}

interface TranslationRequest {
  content: string;
  textbook_id: string;
  content_id?: string;
  content_type: string;
  target_language: string;
}

const API_BASE_URL = process.env.REACT_APP_API_BASE_URL || 'http://localhost:8000';

class ApiClient {
  private baseUrl: string;

  constructor() {
    this.baseUrl = API_BASE_URL;
  }

  // Textbook Generation API
  async createTextbook(request: TextbookCreateRequest): Promise<TextbookResponse> {
    const response = await fetch(`${this.baseUrl}/textbooks`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify(request),
    });

    if (!response.ok) {
      throw new Error(`Failed to create textbook: ${response.statusText}`);
    }

    return response.json();
  }

  async generateTextbookContent(textbookId: string): Promise<any> {
    const response = await fetch(`${this.baseUrl}/textbooks/${textbookId}/generate`, {
      method: 'POST',
    });

    if (!response.ok) {
      throw new Error(`Failed to generate textbook content: ${response.statusText}`);
    }

    return response.json();
  }

  async getTextbook(textbookId: string): Promise<TextbookResponse> {
    const response = await fetch(`${this.baseUrl}/textbooks/${textbookId}`);

    if (!response.ok) {
      throw new Error(`Failed to get textbook: ${response.statusText}`);
    }

    return response.json();
  }

  async getGenerationProgress(textbookId: string): Promise<ProgressResponse> {
    const response = await fetch(`${this.baseUrl}/textbooks/${textbookId}/progress`);

    if (!response.ok) {
      throw new Error(`Failed to get progress: ${response.statusText}`);
    }

    return response.json();
  }

  async customizeTextbookStructure(textbookId: string, structure: any): Promise<any> {
    const response = await fetch(`${this.baseUrl}/textbooks/${textbookId}/structure`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify(structure),
    });

    if (!response.ok) {
      throw new Error(`Failed to customize textbook structure: ${response.statusText}`);
    }

    return response.json();
  }

  async exportTextbook(textbookId: string, format: string): Promise<any> {
    const response = await fetch(`${this.baseUrl}/textbooks/${textbookId}/export`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify({ format, include_navigation: true }),
    });

    if (!response.ok) {
      throw new Error(`Failed to export textbook: ${response.statusText}`);
    }

    return response.json();
  }

  // Chatbot API
  async queryChatbot(request: ChatbotQueryRequest): Promise<any> {
    const response = await fetch(`${this.baseUrl}/chatbot/query`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify(request),
    });

    if (!response.ok) {
      throw new Error(`Failed to query chatbot: ${response.statusText}`);
    }

    return response.json();
  }

  async createChatbotSession(request: ChatbotSessionCreateRequest): Promise<any> {
    const response = await fetch(`${this.baseUrl}/chatbot/sessions`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify(request),
    });

    if (!response.ok) {
      throw new Error(`Failed to create chatbot session: ${response.statusText}`);
    }

    return response.json();
  }

  async indexTextbookForRag(textbookId: string): Promise<any> {
    const response = await fetch(`${this.baseUrl}/chatbot/index-textbook/${textbookId}`, {
      method: 'POST',
    });

    if (!response.ok) {
      throw new Error(`Failed to index textbook for RAG: ${response.statusText}`);
    }

    return response.json();
  }

  // Auth API
  async signup(request: SignupRequest): Promise<any> {
    const response = await fetch(`${this.baseUrl}/auth/signup`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify(request),
    });

    if (!response.ok) {
      throw new Error(`Failed to signup: ${response.statusText}`);
    }

    return response.json();
  }

  async signin(request: SigninRequest): Promise<any> {
    const response = await fetch(`${this.baseUrl}/auth/signin`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify(request),
    });

    if (!response.ok) {
      throw new Error(`Failed to signin: ${response.statusText}`);
    }

    return response.json();
  }

  async getUserProfile(userId: string): Promise<any> {
    const response = await fetch(`${this.baseUrl}/auth/profile/${userId}`);

    if (!response.ok) {
      throw new Error(`Failed to get user profile: ${response.statusText}`);
    }

    return response.json();
  }

  async updateUserProfile(userId: string, profile: any): Promise<any> {
    const response = await fetch(`${this.baseUrl}/auth/profile/${userId}`, {
      method: 'PUT',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify(profile),
    });

    if (!response.ok) {
      throw new Error(`Failed to update user profile: ${response.statusText}`);
    }

    return response.json();
  }

  // Personalization API
  async adaptContent(request: PersonalizationRequest): Promise<any> {
    const response = await fetch(`${this.baseUrl}/personalization/adapt`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify(request),
    });

    if (!response.ok) {
      throw new Error(`Failed to adapt content: ${response.statusText}`);
    }

    return response.json();
  }

  // Translation API
  async translateContent(request: TranslationRequest): Promise<any> {
    const response = await fetch(`${this.baseUrl}/translation/translate`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify(request),
    });

    if (!response.ok) {
      throw new Error(`Failed to translate content: ${response.statusText}`);
    }

    return response.json();
  }
}

export default new ApiClient();