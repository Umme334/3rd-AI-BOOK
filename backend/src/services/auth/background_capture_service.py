from typing import Dict, Any, List, Optional
from datetime import datetime
import re


class BackgroundCaptureService:
    """
    Service for collecting and validating user background information for Physical AI & Humanoid Robotics course
    """

    def __init__(self):
        # Define valid options for background questions
        self.valid_software_levels = ["beginner", "intermediate", "advanced"]
        self.valid_hardware_levels = ["none", "basic", "intermediate", "advanced"]
        self.valid_robotics_experience = ["none", "basic", "intermediate", "advanced"]
        self.valid_math_levels = ["basic", "intermediate", "advanced"]
        self.valid_goals = ["education", "research", "hobby", "career", "other"]
        self.valid_programming_languages = [
            "Python", "C++", "C", "Java", "JavaScript", "TypeScript", "ROS", "MATLAB",
            "Rust", "Go", "Lua", "Assembly", "Other"
        ]
        self.valid_hardware_platforms = [
            "Raspberry Pi", "Arduino", "Jetson", "ROSbot", "TurtleBot", "NVIDIA Isaac",
            "URDF", "Gazebo", "V-REP", "Webots", "Other"
        ]

    def validate_background_info(self, background_data: Dict[str, Any]) -> Dict[str, List[str]]:
        """
        Validate background information and return any errors
        """
        errors = {
            "software_experience": [],
            "hardware_experience": [],
            "programming_languages": [],
            "hardware_platforms": [],
            "robotics_experience": [],
            "math_background": [],
            "primary_goal": [],
            "general": []
        }

        # Validate software experience
        software_exp = background_data.get("software_experience")
        if software_exp and software_exp not in self.valid_software_levels:
            errors["software_experience"].append(
                f"Invalid software experience level. Must be one of: {self.valid_software_levels}"
            )

        # Validate hardware experience
        hardware_exp = background_data.get("hardware_experience")
        if hardware_exp and hardware_exp not in self.valid_hardware_levels:
            errors["hardware_experience"].append(
                f"Invalid hardware experience level. Must be one of: {self.valid_hardware_levels}"
            )

        # Validate programming languages
        prog_langs = background_data.get("programming_languages", [])
        if prog_langs:
            for lang in prog_langs:
                if lang not in self.valid_programming_languages:
                    errors["programming_languages"].append(
                        f"Invalid programming language: {lang}. Must be one of: {self.valid_programming_languages}"
                    )

        # Validate hardware platforms
        hw_platforms = background_data.get("hardware_platforms", [])
        if hw_platforms:
            for platform in hw_platforms:
                if platform not in self.valid_hardware_platforms:
                    errors["hardware_platforms"].append(
                        f"Invalid hardware platform: {platform}. Must be one of: {self.valid_hardware_platforms}"
                    )

        # Validate robotics experience
        robotics_exp = background_data.get("robotics_experience")
        if robotics_exp and robotics_exp not in self.valid_robotics_experience:
            errors["robotics_experience"].append(
                f"Invalid robotics experience level. Must be one of: {self.valid_robotics_experience}"
            )

        # Validate math background
        math_bg = background_data.get("math_background")
        if math_bg and math_bg not in self.valid_math_levels:
            errors["math_background"].append(
                f"Invalid math background level. Must be one of: {self.valid_math_levels}"
            )

        # Validate primary goal
        primary_goal = background_data.get("primary_goal")
        if primary_goal and primary_goal not in self.valid_goals:
            errors["primary_goal"].append(
                f"Invalid primary goal. Must be one of: {self.valid_goals}"
            )

        # Check required fields
        if not software_exp:
            errors["general"].append("Software experience is required")

        return errors

    def sanitize_background_data(self, background_data: Dict[str, Any]) -> Dict[str, Any]:
        """
        Sanitize background data to prevent injection or malicious input
        """
        sanitized_data = {}

        # Sanitize string fields
        string_fields = ["software_experience", "hardware_experience", "robotics_experience",
                        "math_background", "primary_goal"]
        for field in string_fields:
            value = background_data.get(field)
            if value:
                # Remove any potentially harmful characters
                sanitized_value = str(value).strip()
                # Only allow alphanumeric, spaces, hyphens, and underscores
                if re.match(r'^[a-zA-Z0-9\s\-_]+$', sanitized_value):
                    sanitized_data[field] = sanitized_value
                else:
                    sanitized_data[field] = re.sub(r'[^a-zA-Z0-9\s\-_]', '', sanitized_value)

        # Sanitize list fields
        list_fields = ["programming_languages", "hardware_platforms"]
        for field in list_fields:
            values = background_data.get(field, [])
            if values:
                sanitized_list = []
                for value in values:
                    # Sanitize each value in the list
                    sanitized_value = str(value).strip()
                    if re.match(r'^[a-zA-Z0-9\s\-_]+$', sanitized_value):
                        sanitized_list.append(sanitized_value)
                    else:
                        clean_value = re.sub(r'[^a-zA-Z0-9\s\-_]', '', sanitized_value)
                        if clean_value:
                            sanitized_list.append(clean_value)
                sanitized_data[field] = sanitized_list

        # Sanitize background questions (arbitrary key-value pairs)
        background_questions = background_data.get("background_questions", {})
        if background_questions:
            sanitized_questions = {}
            for key, value in background_questions.items():
                # Sanitize key
                if re.match(r'^[a-zA-Z0-9_]+$', str(key)):
                    clean_key = str(key)
                else:
                    clean_key = re.sub(r'[^a-zA-Z0-9_]', '', str(key))

                # Sanitize value
                if isinstance(value, str):
                    if re.match(r'^[a-zA-Z0-9\s\-_,.!?]+$', value):
                        sanitized_questions[clean_key] = value
                    else:
                        sanitized_questions[clean_key] = re.sub(r'[^a-zA-Z0-9\s\-_,.!?]', '', value)
                else:
                    sanitized_questions[clean_key] = value

            sanitized_data["background_questions"] = sanitized_questions

        return sanitized_data

    def extract_education_relevance(self, background_data: Dict[str, Any]) -> Dict[str, Any]:
        """
        Extract information relevant for educational personalization
        """
        relevance_info = {
            "programming_readiness": self._assess_programming_readiness(
                background_data.get("software_experience"),
                background_data.get("programming_languages", [])
            ),
            "hardware_readiness": self._assess_hardware_readiness(
                background_data.get("hardware_experience"),
                background_data.get("hardware_platforms", [])
            ),
            "robotics_readiness": self._assess_robotics_readiness(
                background_data.get("robotics_experience")
            ),
            "math_readiness": self._assess_math_readiness(
                background_data.get("math_background")
            ),
            "learning_goals": background_data.get("primary_goal"),
            "recommended_path": self._determine_learning_path(
                background_data.get("software_experience"),
                background_data.get("hardware_experience"),
                background_data.get("robotics_experience"),
                background_data.get("primary_goal")
            )
        }

        return relevance_info

    def _assess_programming_readiness(self, software_exp: str, programming_langs: List[str]) -> str:
        """
        Assess programming readiness based on background
        """
        if not software_exp:
            return "unknown"

        # Base assessment on software experience
        if software_exp == "beginner":
            readiness = "basic"
        elif software_exp == "intermediate":
            readiness = "intermediate"
        elif software_exp == "advanced":
            readiness = "advanced"
        else:
            readiness = "basic"

        # Adjust based on programming languages known
        python_known = any(lang.lower() in ["python", "ros"] for lang in programming_langs)
        c_cpp_known = any(lang.lower() in ["c++", "c"] for lang in programming_langs)

        if python_known and readiness == "basic":
            readiness = "intermediate"
        elif c_cpp_known and readiness == "basic":
            readiness = "intermediate"
        elif python_known and c_cpp_known and readiness == "intermediate":
            readiness = "advanced"

        return readiness

    def _assess_hardware_readiness(self, hardware_exp: str, hardware_platforms: List[str]) -> str:
        """
        Assess hardware readiness based on background
        """
        if not hardware_exp:
            return "unknown"

        # Base assessment on hardware experience
        if hardware_exp == "none":
            readiness = "none"
        elif hardware_exp == "basic":
            readiness = "basic"
        elif hardware_exp == "intermediate":
            readiness = "intermediate"
        elif hardware_exp == "advanced":
            readiness = "advanced"
        else:
            readiness = "basic"

        # Adjust based on hardware platforms known
        has_robotics_platforms = any(platform.lower() in ["rosbot", "turtlebot", "nvidia isaac", "urdf", "gazebo"] for platform in hardware_platforms)
        has_dev_platforms = any(platform.lower() in ["raspberry pi", "arduino", "jetson"] for platform in hardware_platforms)

        if has_robotics_platforms and readiness == "basic":
            readiness = "intermediate"
        elif has_dev_platforms and readiness == "none":
            readiness = "basic"
        elif has_robotics_platforms and readiness == "intermediate":
            readiness = "advanced"

        return readiness

    def _assess_robotics_readiness(self, robotics_exp: str) -> str:
        """
        Assess robotics readiness based on background
        """
        if not robotics_exp:
            return "none"

        return robotics_exp

    def _assess_math_readiness(self, math_bg: str) -> str:
        """
        Assess math readiness based on background
        """
        if not math_bg:
            return "basic"

        return math_bg

    def _determine_learning_path(self, software_exp: str, hardware_exp: str, robotics_exp: str, primary_goal: str) -> str:
        """
        Determine recommended learning path based on background and goals
        """
        # Default path
        path = "comprehensive"

        # Adjust path based on experience
        if software_exp == "advanced" and hardware_exp == "advanced" and robotics_exp == "advanced":
            path = "accelerated"
        elif software_exp == "beginner" and hardware_exp == "none" and robotics_exp == "none":
            path = "foundational"
        elif primary_goal == "career":
            path = "career-focused"
        elif primary_goal == "research":
            path = "research-focused"

        return path

    def generate_personalization_profile(self, background_data: Dict[str, Any]) -> Dict[str, Any]:
        """
        Generate a personalization profile based on background information
        """
        # Validate the background data first
        errors = self.validate_background_info(background_data)
        has_errors = any(errors.values())

        if has_errors:
            raise ValueError(f"Invalid background data: {errors}")

        # Sanitize the data
        sanitized_data = self.sanitize_background_data(background_data)

        # Extract relevance information
        relevance_info = self.extract_education_relevance(sanitized_data)

        # Create personalization profile
        profile = {
            "created_at": datetime.now(),
            "updated_at": datetime.now(),
            "background_data": sanitized_data,
            "relevance_info": relevance_info,
            "content_preferences": {
                "difficulty_adaptation": self._determine_difficulty_adaptation(
                    sanitized_data.get("software_experience"),
                    sanitized_data.get("hardware_experience"),
                    sanitized_data.get("robotics_experience")
                ),
                "content_pacing": self._determine_content_pacing(
                    sanitized_data.get("primary_goal")
                ),
                "example_preference": self._determine_example_preference(
                    sanitized_data.get("programming_languages", []),
                    sanitized_data.get("hardware_platforms", [])
                )
            },
            "recommended_modules": self._determine_recommended_modules(
                sanitized_data.get("software_experience"),
                sanitized_data.get("hardware_experience"),
                sanitized_data.get("robotics_experience"),
                sanitized_data.get("primary_goal")
            )
        }

        return profile

    def _determine_difficulty_adaptation(self, software_exp: str, hardware_exp: str, robotics_exp: str) -> str:
        """
        Determine how content difficulty should be adapted
        """
        # Base difficulty on lowest experience level
        exp_levels = [software_exp, hardware_exp, robotics_exp]
        exp_scores = {
            "beginner": 1,
            "intermediate": 2,
            "advanced": 3,
            "none": 0,
            "basic": 1
        }

        min_score = min([exp_scores.get(exp, 0) for exp in exp_levels if exp])

        if min_score >= 3:
            return "advanced"
        elif min_score >= 2:
            return "intermediate"
        else:
            return "beginner"

    def _determine_content_pacing(self, primary_goal: str) -> str:
        """
        Determine content pacing based on primary goal
        """
        if primary_goal in ["career", "research"]:
            return "accelerated"
        elif primary_goal == "hobby":
            return "self_paced"
        else:
            return "standard"

    def _determine_example_preference(self, programming_langs: List[str], hardware_platforms: List[str]) -> Dict[str, str]:
        """
        Determine preferred examples based on known languages and platforms
        """
        example_pref = {
            "programming": "python",
            "platform": "gazebo"
        }

        if "C++" in programming_langs or "C" in programming_langs:
            example_pref["programming"] = "cpp"
        elif "ROS" in programming_langs:
            example_pref["programming"] = "ros"
        elif "MATLAB" in programming_langs:
            example_pref["programming"] = "matlab"

        if "NVIDIA Isaac" in hardware_platforms:
            example_pref["platform"] = "nvidia-isaac"
        elif "ROSbot" in hardware_platforms:
            example_pref["platform"] = "rosbot"
        elif "TurtleBot" in hardware_platforms:
            example_pref["platform"] = "turtlebot"
        elif "Webots" in hardware_platforms:
            example_pref["platform"] = "webots"

        return example_pref

    def _determine_recommended_modules(self, software_exp: str, hardware_exp: str, robotics_exp: str, primary_goal: str) -> List[str]:
        """
        Determine recommended modules based on background and goals
        """
        modules = []

        # Base modules for all users
        modules.extend(["introduction", "ethics"])

        # Add modules based on experience
        if software_exp in ["beginner", "intermediate", "advanced"]:
            modules.append("programming-fundamentals")

        if hardware_exp in ["basic", "intermediate", "advanced"]:
            modules.append("hardware-basics")

        if robotics_exp in ["basic", "intermediate", "advanced"]:
            modules.append("robotics-fundamentals")

        # Add advanced modules based on experience
        if software_exp in ["intermediate", "advanced"]:
            modules.append("advanced-programming")

        if hardware_exp in ["intermediate", "advanced"]:
            modules.append("advanced-hardware")

        if robotics_exp in ["intermediate", "advanced"]:
            modules.append("advanced-robotics")

        # Add goal-specific modules
        if primary_goal == "research":
            modules.extend(["research-methods", "advanced-topics"])
        elif primary_goal == "career":
            modules.extend(["industry-practices", "career-skills"])
        elif primary_goal == "hobby":
            modules.append("project-based-learning")

        # Add Physical AI specific modules
        modules.extend([
            "physical-ai-concepts",
            "embodied-intelligence",
            "ros-integration",
            "gazebo-simulation"
        ])

        # Add NVIDIA Isaac if hardware experience is advanced
        if hardware_exp == "advanced":
            modules.append("nvidia-isaac-platform")

        return list(set(modules))  # Remove duplicates