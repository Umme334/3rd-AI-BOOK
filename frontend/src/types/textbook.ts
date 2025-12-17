export interface TextbookCreateRequest {
  title: string;
  subject: string;
  difficulty: string;
  target_audience?: string;
  chapter_count: number;
  include_quizzes: boolean;
  include_summaries: boolean;
  course_module?: string;
  hardware_requirements?: string[];
  enable_personalization: boolean;
  enable_translation: boolean;
}

export interface TextbookResponse {
  id: string;
  title: string;
  subject: string;
  difficulty: string;
  targetAudience: string;
  chapters: Chapter[];
  status: string;
  createdAt: string;
}

export interface Chapter {
  id: string;
  title: string;
  sections: Section[];
  metadata: Record<string, any>;
}

export interface Section {
  id: string;
  title: string;
  content: string;
  contentType: string;
  metadata: Record<string, any>;
}

export interface ProgressResponse {
  textbookId: string;
  progress: number;
  status: string;
  message: string;
}

export interface ExportRequest {
  format: string;
  include_navigation?: boolean;
}