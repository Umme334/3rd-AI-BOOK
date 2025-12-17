export interface PersonalizationRequest {
  content: string;
  user_id: string;
  content_type: string; // 'textbook', 'chapter', 'section', 'interactive_element'
}

export interface PersonalizationResponse {
  original_content: string;
  adapted_content: string;
  user_profile_used: string;
  content_type: string;
  adaptation_metadata: {
    difficulty_level: string;
    technical_depth: string;
    examples_included: boolean;
  };
}

export interface UserPreferences {
  user_id: string;
  software_experience: string;
  hardware_experience: string;
  programming_languages: string[];
  hardware_platforms: string[];
  robotics_experience: string;
  math_background: string;
  primary_goal: string;
  preferred_difficulty: string;
  learning_style: string;
  update_preferences: (preferences: Partial<UserPreferences>) => void;
}