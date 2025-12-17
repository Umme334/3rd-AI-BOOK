export interface User {
  id: string;
  email: string;
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
  createdAt: string;
  updatedAt: string;
}

export interface SignupRequest {
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

export interface SigninRequest {
  email: string;
  password: string;
}

export interface AuthResponse {
  user_id: string;
  email: string;
  access_token: string;
  refresh_token?: string;
  token_type: string;
  expires_at: string;
  user_profile: User;
}