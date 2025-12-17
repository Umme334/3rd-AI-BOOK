export interface TextbookCreateRequest {
  title: string;
  subject: string;
  difficulty: DifficultyLevel;
  target_audience?: string;
  chapter_count?: number;
  include_quizzes: boolean;
  include_summaries: boolean;
  // Physical AI specific attributes
  course_module?: string; // e.g., "ROS 2", "Gazebo", "NVIDIA Isaac", "Vision-Language-Action"
  hardware_requirements?: string[];
  enable_personalization?: boolean;
  enable_translation?: boolean;
}

export interface TextbookResponse {
  id: string;
  title: string;
  subject: string;
  difficulty: DifficultyLevel;
  target_audience?: string;
  chapters: ChapterResponse[];
  metadata: Record<string, any>;
  export_formats: string[];
  status: TextbookStatus;
  created_at: string;
  updated_at: string;
  // Physical AI specific attributes
  course_module?: string;
  hardware_requirements: string[];
  learning_outcomes: string[];
  rag_indexed: boolean;
  personalization_enabled: boolean;
  available_translations: string[];
}

export interface ChapterResponse {
  id: string;
  title: string;
  position: number;
  sections: SectionResponse[];
  word_count: number;
  learning_objectives: string[];
  // Physical AI specific attributes
  module_type: string; // e.g., "ROS 2", "Gazebo", "NVIDIA Isaac", "Vision-Language-Action"
  hardware_focus: string[]; // e.g., ["simulation", "real-hardware", "cloud-deployment"]
  difficulty_level: DifficultyLevel;
  estimated_duration: string; // e.g., "2 weeks", "3 hours", "1 module"
  prerequisites: string[]; // e.g., ["Python basics", "Linux command line"]
  // Personalization support
  personalized_content?: Record<string, any>; // Content adapted for different user backgrounds
  // Translation support
  translated_titles?: Record<string, string>; // {language_code: translated_title}
}

export interface SectionResponse {
  id: string;
  title: string;
  content: string;
  position: number;
  interactive_elements: InteractiveElementResponse[];
  key_terms: string[];
  // Physical AI specific attributes
  topic_category: string; // e.g., "theoretical", "practical", "simulation", "hardware"
  hardware_requirements: string[]; // Specific hardware needed for this section
  code_examples: string[]; // List of code example IDs or content
  // Personalization support
  personalized_content?: Record<string, any>; // Content adapted for different user backgrounds
  difficulty_adjustments?: Record<string, any>; // Difficulty adjustments based on user background
  // Translation support
  translated_content?: Record<string, string>; // {language_code: translated_content}
}

export interface InteractiveElementResponse {
  id: string;
  type: string; // quiz, summary, booster, exercise, simulation, code-challenge
  content: string;
  position: number;
  metadata?: Record<string, any>;
  // Physical AI specific attributes
  hardware_relevance: string[]; // e.g., ["simulation", "real-robot", "cloud"]
  difficulty_level: DifficultyLevel;
  learning_objective: string; // Specific learning objective for this element
  // Personalization support
  personalized_variants?: Record<string, any>; // Different versions based on user background
  // Translation support
  translated_content?: Record<string, string>; // {language_code: translated_content}
}

export interface ProgressResponse {
  id: string;
  status: TextbookStatus;
  progress: number; // 0.0 to 1.0
  message?: string;
}

export interface UserProfile {
  id: string;
  email: string;
  name: string;
  software_background: string; // e.g., "beginner", "intermediate", "advanced" in programming
  hardware_background: string; // e.g., "none", "basic", "advanced" in robotics/hardware
  programming_languages: string[];
  hardware_experience: string[]; // e.g., ["raspberry pi", "arduino", "jetson"]
  robotics_experience: string; // e.g., "none", "basic", "intermediate", "advanced"
  education_level: string; // e.g., "undergraduate", "graduate", "professional"
  primary_language: string;
  preferences: Record<string, any>;
  created_at: string;
  updated_at: string;
}

export interface ChatbotMessage {
  id: string;
  role: string; // "user" or "assistant"
  content: string;
  timestamp: string;
  metadata?: Record<string, any>;
}

export interface ChatbotSession {
  id: string;
  user_id?: string;
  textbook_id: string;
  messages: ChatbotMessage[];
  context_chunks: Record<string, any>[]; // Retrieved context from RAG
  created_at: string;
  updated_at: string;
  is_active: boolean;
}

export interface ChatbotQueryRequest {
  query: string;
  textbook_id: string;
  session_id?: string;
}

export interface ChatbotResponse {
  response: string;
  session_id: string;
  context_sources: string[];
  followup_questions?: string[];
}

export interface PersonalizationRequest {
  textbook_id: string;
  user_id: string;
  content_type: string; // "textbook", "chapter", "section"
  content_id?: string;
}

export interface TranslationRequest {
  content_id: string;
  content_type: string; // "textbook", "chapter", "section", "interactive_element"
  target_language: string; // e.g., "ur" for Urdu
  textbook_id: string;
}

export type DifficultyLevel = 'beginner' | 'intermediate' | 'advanced';
export type TextbookStatus = 'draft' | 'generating' | 'complete' | 'failed';