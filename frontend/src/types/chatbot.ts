export interface ChatbotQueryRequest {
  textbook_id: string;
  query: string;
  session_id?: string;
  selected_text?: string;
}

export interface ChatbotResponse {
  response: string;
  session_id: string;
  sources: Array<Record<string, any>>;
  context_sources: string[];
  followup_questions: string[];
  query_time: string;
  confidence: number;
  tokens_used?: number;
}

export interface ChatbotSessionCreateRequest {
  textbook_id: string;
  user_id?: string;
  session_name?: string;
}

export interface ChatbotSessionResponse {
  session_id: string;
  textbook_id: string;
  user_id?: string;
  session_name: string;
  created_at: string;
  last_accessed: string;
  message_count: number;
  active: boolean;
}

export interface ChatMessage {
  id: string;
  role: 'user' | 'assistant';
  content: string;
  timestamp: string;
  sources?: string[];
}

export interface ChatHistoryResponse {
  session_id: string;
  messages: ChatMessage[];
  total_messages: number;
}

export interface ChatbotSession {
  id: string;
  textbook_id: string;
  user_id?: string;
  messages: ChatMessage[];
  context_chunks?: any[];
  is_active: boolean;
  created_at?: string;
  updated_at?: string;
}