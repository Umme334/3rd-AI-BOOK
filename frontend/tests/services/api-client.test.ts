import ApiClient from '../../src/services/api-client';

// Mock the fetch API
global.fetch = jest.fn();

describe('ApiClient Service', () => {
  let apiClient: ApiClient;

  beforeEach(() => {
    apiClient = new ApiClient();
    (global.fetch as jest.MockedFunction<typeof global.fetch>).mockClear();
  });

  describe('Textbook Generation API', () => {
    it('should create a textbook', async () => {
      const mockResponse = {
        success: true,
        data: {
          id: 'textbook-123',
          title: 'Test Textbook',
          subject: 'Physical AI',
          status: 'created'
        }
      };

      (global.fetch as jest.MockedFunction<typeof global.fetch>).mockResolvedValueOnce({
        json: () => Promise.resolve(mockResponse),
        ok: true,
      } as Response);

      const textbookData = {
        subject: 'Physical AI',
        difficulty: 'intermediate',
        targetAudience: 'students',
        length: 5
      };

      const result = await apiClient.createTextbook(textbookData);

      expect(result).toEqual(mockResponse);
      expect(global.fetch).toHaveBeenCalledWith(
        'http://localhost:8000/textbooks',
        expect.objectContaining({
          method: 'POST',
          headers: {
            'Content-Type': 'application/json',
          },
          body: JSON.stringify(textbookData),
        })
      );
    });

    it('should generate textbook content', async () => {
      const mockResponse = {
        success: true,
        data: {
          textbookId: 'textbook-123',
          status: 'generating',
          progress: 0
        }
      };

      (global.fetch as jest.MockedFunction<typeof global.fetch>).mockResolvedValueOnce({
        json: () => Promise.resolve(mockResponse),
        ok: true,
      } as Response);

      const result = await apiClient.generateTextbook(textbookId);

      expect(result).toEqual(mockResponse);
      expect(global.fetch).toHaveBeenCalledWith(
        'http://localhost:8000/textbooks/textbook-123/generate',
        expect.objectContaining({
          method: 'POST',
        })
      );
    });

    it('should get textbook details', async () => {
      const textbookId = 'textbook-123';
      const mockResponse = {
        success: true,
        data: {
          id: textbookId,
          title: 'Test Textbook',
          subject: 'Physical AI',
          chapters: []
        }
      };

      (global.fetch as jest.MockedFunction<typeof global.fetch>).mockResolvedValueOnce({
        json: () => Promise.resolve(mockResponse),
        ok: true,
      } as Response);

      const result = await apiClient.getTextbook(textbookId);

      expect(result).toEqual(mockResponse);
      expect(global.fetch).toHaveBeenCalledWith(
        'http://localhost:8000/textbooks/textbook-123'
      );
    });

    it('should get generation progress', async () => {
      const textbookId = 'textbook-123';
      const mockResponse = {
        success: true,
        data: {
          textbookId,
          progress: 75,
          status: 'generating',
          message: 'Generating chapter 3 of 5'
        }
      };

      (global.fetch as jest.MockedFunction<typeof global.fetch>).mockResolvedValueOnce({
        json: () => Promise.resolve(mockResponse),
        ok: true,
      } as Response);

      const result = await apiClient.getTextbookProgress(textbookId);

      expect(result).toEqual(mockResponse);
      expect(global.fetch).toHaveBeenCalledWith(
        'http://localhost:8000/textbooks/textbook-123/progress'
      );
    });

    it('should customize textbook structure', async () => {
      const textbookId = 'textbook-123';
      const structureData = {
        chapters: [
          {
            id: 'chapter-1',
            title: 'Introduction',
            sections: [
              {
                id: 'section-1',
                title: 'Overview',
                contentTypes: ['text', 'images']
              }
            ]
          }
        ]
      };
      const mockResponse = {
        success: true,
        data: {
          textbookId,
          structure: structureData,
          message: 'Structure updated successfully'
        }
      };

      (global.fetch as jest.MockedFunction<typeof global.fetch>).mockResolvedValueOnce({
        json: () => Promise.resolve(mockResponse),
        ok: true,
      } as Response);

      const result = await apiClient.customizeTextbookStructure(textbookId, structureData);

      expect(result).toEqual(mockResponse);
      expect(global.fetch).toHaveBeenCalledWith(
        'http://localhost:8000/textbooks/textbook-123/structure',
        expect.objectContaining({
          method: 'POST',
          headers: {
            'Content-Type': 'application/json',
          },
          body: JSON.stringify(structureData),
        })
      );
    });

    it('should export textbook', async () => {
      const textbookId = 'textbook-123';
      const exportData = { format: 'pdf' };
      const mockResponse = {
        success: true,
        data: {
          textbookId,
          format: 'pdf',
          downloadUrl: `/api/textbooks/textbook-123/export/pdf`,
          message: 'Export completed successfully'
        }
      };

      (global.fetch as jest.MockedFunction<typeof global.fetch>).mockResolvedValueOnce({
        json: () => Promise.resolve(mockResponse),
        ok: true,
      } as Response);

      const result = await apiClient.exportTextbook(textbookId, exportData);

      expect(result).toEqual(mockResponse);
      expect(global.fetch).toHaveBeenCalledWith(
        'http://localhost:8000/textbooks/textbook-123/export',
        expect.objectContaining({
          method: 'POST',
          headers: {
            'Content-Type': 'application/json',
          },
          body: JSON.stringify({ format: 'pdf', include_navigation: true }),
        })
      );
    });
  });

  describe('Chatbot API', () => {
    it('should query the chatbot', async () => {
      const queryData = {
        textbook_id: 'textbook-123',
        query: 'What is ROS 2?',
        session_id: 'session-456'
      };
      const mockResponse = {
        success: true,
        data: {
          response: 'ROS 2 is a robotics middleware...',
          session_id: 'session-456',
          context_sources: ['textbook_content'],
          followup_questions: ['What else would you like to know?'],
          query_time: new Date().toISOString(),
          confidence: 0.85
        }
      };

      (global.fetch as jest.MockedFunction<typeof global.fetch>).mockResolvedValueOnce({
        json: () => Promise.resolve(mockResponse),
        ok: true,
      } as Response);

      const result = await apiClient.queryChatbot(queryData);

      expect(result).toEqual(mockResponse);
      expect(global.fetch).toHaveBeenCalledWith(
        'http://localhost:8000/chatbot/query',
        expect.objectContaining({
          method: 'POST',
          headers: {
            'Content-Type': 'application/json',
          },
          body: JSON.stringify(queryData),
        })
      );
    });

    it('should create a chatbot session', async () => {
      const sessionData = {
        textbook_id: 'textbook-123',
        user_id: 'user-789',
        session_name: 'My Session'
      };
      const mockResponse = {
        success: true,
        data: {
          session_id: 'session-456',
          textbook_id: 'textbook-123',
          user_id: 'user-789',
          session_name: 'My Session',
          created_at: new Date().toISOString(),
          last_accessed: new Date().toISOString(),
          message_count: 0,
          active: true
        }
      };

      (global.fetch as jest.MockedFunction<typeof global.fetch>).mockResolvedValueOnce({
        json: () => Promise.resolve(mockResponse),
        ok: true,
      } as Response);

      const result = await apiClient.createChatbotSession(sessionData);

      expect(result).toEqual(mockResponse);
      expect(global.fetch).toHaveBeenCalledWith(
        'http://localhost:8000/chatbot/sessions',
        expect.objectContaining({
          method: 'POST',
          headers: {
            'Content-Type': 'application/json',
          },
          body: JSON.stringify(sessionData),
        })
      );
    });

    it('should index textbook for RAG', async () => {
      const textbookId = 'textbook-123';
      const mockResponse = {
        success: true,
        data: {
          message: `Successfully indexed textbook ${textbookId} for RAG`,
          textbook_id: textbookId,
          indexed_at: new Date().toISOString()
        }
      };

      (global.fetch as jest.MockedFunction<typeof global.fetch>).mockResolvedValueOnce({
        json: () => Promise.resolve(mockResponse),
        ok: true,
      } as Response);

      const result = await apiClient.indexTextbookForRag(textbookId);

      expect(result).toEqual(mockResponse);
      expect(global.fetch).toHaveBeenCalledWith(
        `http://localhost:8000/chatbot/index-textbook/${textbookId}`,
        expect.objectContaining({
          method: 'POST',
        })
      );
    });
  });

  describe('Auth API', () => {
    it('should signup a user', async () => {
      const userData = {
        email: 'test@example.com',
        password: 'securepassword123',
        firstName: 'John',
        lastName: 'Doe',
        softwareExperience: 'intermediate',
        hardwareExperience: 'beginner'
      };
      const mockResponse = {
        success: true,
        data: {
          user_id: 'user-123',
          email: 'test@example.com',
          access_token: 'token-456',
          token_type: 'Bearer',
          profile_created: true,
          message: 'User registered successfully'
        }
      };

      (global.fetch as jest.MockedFunction<typeof global.fetch>).mockResolvedValueOnce({
        json: () => Promise.resolve(mockResponse),
        ok: true,
      } as Response);

      const result = await apiClient.signup(userData);

      expect(result).toEqual(mockResponse);
      expect(global.fetch).toHaveBeenCalledWith(
        'http://localhost:8000/auth/signup',
        expect.objectContaining({
          method: 'POST',
          headers: {
            'Content-Type': 'application/json',
          },
          body: JSON.stringify(userData),
        })
      );
    });

    it('should signin a user', async () => {
      const credentials = {
        email: 'test@example.com',
        password: 'securepassword123'
      };
      const mockResponse = {
        success: true,
        data: {
          user_id: 'user-123',
          email: 'test@example.com',
          access_token: 'token-456',
          refresh_token: 'refresh-789',
          token_type: 'Bearer',
          expires_at: new Date(Date.now() + 3600000).toISOString(),
          user_profile: {
            user_id: 'user-123',
            email: 'test@example.com',
            software_experience: 'intermediate',
            hardware_experience: 'beginner'
          }
        }
      };

      (global.fetch as jest.MockedFunction<typeof global.fetch>).mockResolvedValueOnce({
        json: () => Promise.resolve(mockResponse),
        ok: true,
      } as Response);

      const result = await apiClient.signin(credentials);

      expect(result).toEqual(mockResponse);
      expect(global.fetch).toHaveBeenCalledWith(
        'http://localhost:8000/auth/signin',
        expect.objectContaining({
          method: 'POST',
          headers: {
            'Content-Type': 'application/json',
          },
          body: JSON.stringify(credentials),
        })
      );
    });

    it('should get user profile', async () => {
      const userId = 'user-123';
      const mockResponse = {
        success: true,
        data: {
          user_id: userId,
          email: 'test@example.com',
          software_experience: 'intermediate',
          hardware_experience: 'beginner'
        }
      };

      (global.fetch as jest.MockedFunction<typeof global.fetch>).mockResolvedValueOnce({
        json: () => Promise.resolve(mockResponse),
        ok: true,
      } as Response);

      const result = await apiClient.getUserProfile(userId);

      expect(result).toEqual(mockResponse);
      expect(global.fetch).toHaveBeenCalledWith(
        `http://localhost:8000/auth/profile/${userId}`
      );
    });

    it('should update user profile', async () => {
      const userId = 'user-123';
      const profileData = {
        software_experience: 'advanced',
        primary_goal: 'Research in robotics'
      };
      const mockResponse = {
        success: true,
        data: {
          user_id: userId,
          software_experience: 'advanced',
          primary_goal: 'Research in robotics',
          updated_at: new Date().toISOString()
        }
      };

      (global.fetch as jest.MockedFunction<typeof global.fetch>).mockResolvedValueOnce({
        json: () => Promise.resolve(mockResponse),
        ok: true,
      } as Response);

      const result = await apiClient.updateUserProfile(userId, profileData);

      expect(result).toEqual(mockResponse);
      expect(global.fetch).toHaveBeenCalledWith(
        `http://localhost:8000/auth/profile/${userId}`,
        expect.objectContaining({
          method: 'PUT',
          headers: {
            'Content-Type': 'application/json',
          },
          body: JSON.stringify(profileData),
        })
      );
    });
  });

  describe('Personalization API', () => {
    it('should adapt content', async () => {
      const contentData = {
        content: 'Original content',
        user_id: 'user-123',
        content_type: 'chapter'
      };
      const mockResponse = {
        success: true,
        data: {
          original_content: 'Original content',
          adapted_content: '[Personalized for your background] Original content',
          user_profile_used: 'user-123',
          content_type: 'chapter'
        }
      };

      (global.fetch as jest.MockedFunction<typeof global.fetch>).mockResolvedValueOnce({
        json: () => Promise.resolve(mockResponse),
        ok: true,
      } as Response);

      const result = await apiClient.adaptContent(contentData);

      expect(result).toEqual(mockResponse);
      expect(global.fetch).toHaveBeenCalledWith(
        'http://localhost:8000/personalization/adapt',
        expect.objectContaining({
          method: 'POST',
          headers: {
            'Content-Type': 'application/json',
          },
          body: JSON.stringify(contentData),
        })
      );
    });
  });

  describe('Translation API', () => {
    it('should translate content', async () => {
      const translationData = {
        content: 'Hello, this is a test',
        textbook_id: 'textbook-123',
        content_id: 'section-456',
        content_type: 'text',
        target_language: 'ur'
      };
      const mockResponse = {
        success: true,
        data: {
          original_content: 'Hello, this is a test',
          translated_content: '[UR] Hello, this is a test',
          source_language: 'en',
          target_language: 'ur',
          content_type: 'text'
        }
      };

      (global.fetch as jest.MockedFunction<typeof global.fetch>).mockResolvedValueOnce({
        json: () => Promise.resolve(mockResponse),
        ok: true,
      } as Response);

      const result = await apiClient.translateContent(translationData);

      expect(result).toEqual(mockResponse);
      expect(global.fetch).toHaveBeenCalledWith(
        'http://localhost:8000/translation/translate',
        expect.objectContaining({
          method: 'POST',
          headers: {
            'Content-Type': 'application/json',
          },
          body: JSON.stringify(translationData),
        })
      );
    });
  });

  describe('Error Handling', () => {
    it('should handle API errors', async () => {
      (global.fetch as jest.MockedFunction<typeof global.fetch>).mockResolvedValueOnce({
        ok: false,
        status: 500,
        statusText: 'Internal Server Error'
      } as Response);

      const result = await apiClient.getTextbook('textbook-123');

      expect(result.success).toBe(false);
      expect(result.error).toBeDefined();
    });
  });
});