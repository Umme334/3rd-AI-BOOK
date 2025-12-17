#!/usr/bin/env python3
"""
Quickstart validation script to verify all components are working properly
"""

import subprocess
import sys
import os
import time
from pathlib import Path


def check_python_version():
    """Check if Python 3.8+ is available"""
    if sys.version_info < (3, 8):
        print("[ERROR] Python 3.8 or higher is required")
        return False
    print("[SUCCESS] Python version is compatible")
    return True


def check_node_version():
    """Check if Node.js is available"""
    try:
        result = subprocess.run(["node", "--version"], capture_output=True, text=True)
        if result.returncode == 0:
            print(f"[SUCCESS] Node.js version: {result.stdout.strip()}")
            return True
        else:
            print("[ERROR] Node.js is not installed or not in PATH")
            return False
    except FileNotFoundError:
        print("[ERROR] Node.js is not installed or not in PATH")
        return False


def check_npm_version():
    """Check if npm is available"""
    try:
        result = subprocess.run(["npm", "--version"], capture_output=True, text=True)
        if result.returncode == 0:
            print(f"[SUCCESS] npm version: {result.stdout.strip()}")
            return True
        else:
            print("[ERROR] npm is not installed or not in PATH")
            return False
    except FileNotFoundError:
        print("[ERROR] npm is not installed or not in PATH")
        return False


def check_dependencies_installed():
    """Check if backend dependencies are installed"""
    backend_path = Path("backend")
    if not backend_path.exists():
        print("[ERROR] Backend directory not found")
        return False

    try:
        # Try to import key packages
        import fastapi
        import pydantic
        import openai
        print("[SUCCESS] Backend dependencies are available")
        return True
    except ImportError as e:
        print(f"[ERROR] Backend dependencies missing: {e}")
        return False


def check_frontend_dependencies():
    """Check if frontend dependencies are installed"""
    frontend_path = Path("frontend")
    if not frontend_path.exists():
        print("[ERROR] Frontend directory not found")
        return False

    package_json = frontend_path / "package.json"
    if not package_json.exists():
        print("[ERROR] package.json not found in frontend")
        return False

    node_modules = frontend_path / "node_modules"
    if not node_modules.exists():
        print("[WARNING] Frontend dependencies not installed (node_modules missing)")
        return True  # Not a failure, just a warning

    print("[SUCCESS] Frontend dependencies are installed")
    return True


def validate_structure():
    """Validate project structure"""
    required_dirs = [
        "backend",
        "frontend",
        "backend/src",
        "backend/src/api",
        "backend/src/services",
        "backend/src/models",
        "backend/src/schemas",
        "frontend/src",
        "frontend/src/components",
        "specs",
        "docs"
    ]

    missing_dirs = []
    for dir_path in required_dirs:
        if not Path(dir_path).exists():
            missing_dirs.append(dir_path)

    if missing_dirs:
        print(f"[ERROR] Missing directories: {missing_dirs}")
        return False

    print("[SUCCESS] Project structure is valid")
    return True


def validate_config_files():
    """Validate configuration files exist"""
    required_files = [
        "backend/src/config.py",
        "frontend/.env.example",
        "specs/1-textbook-generation/spec.md",
        "specs/1-textbook-generation/plan.md",
        "specs/1-textbook-generation/tasks.md"
    ]

    missing_files = []
    for file_path in required_files:
        if not Path(file_path).exists():
            missing_files.append(file_path)

    if missing_files:
        print(f"[ERROR] Missing configuration files: {missing_files}")
        return False

    print("[SUCCESS] Configuration files are present")
    return True


def run_tests():
    """Check if test files exist and can be discovered"""
    backend_tests = list(Path("backend").rglob("test_*.py")) + list(Path("backend").rglob("*_test.py"))
    frontend_tests = list(Path("frontend").rglob("*.test.*")) + list(Path("frontend").rglob("*test.*"))

    if not backend_tests:
        print("[WARNING] No backend test files found")
    else:
        print(f"[SUCCESS] Found {len(backend_tests)} backend test files")

    if not frontend_tests:
        print("[WARNING] No frontend test files found")
    else:
        print(f"[SUCCESS] Found {len(frontend_tests)} frontend test files")

    return True


def main():
    """Main validation function"""
    print("Starting quickstart validation...")
    print("="*50)

    checks = [
        ("Python Version", check_python_version),
        ("Node.js Version", check_node_version),
        ("npm Version", check_npm_version),
        ("Backend Dependencies", check_dependencies_installed),
        ("Frontend Dependencies", check_frontend_dependencies),
        ("Project Structure", validate_structure),
        ("Configuration Files", validate_config_files),
        ("Test Files", run_tests),
    ]

    passed = 0
    total = len(checks)

    for check_name, check_func in checks:
        print(f"\n[INFO] {check_name}...")
        if check_func():
            passed += 1
        else:
            print(f"[ERROR] {check_name} failed")

    print("\n" + "="*50)
    print(f"[SUMMARY] Validation Summary: {passed}/{total} checks passed")

    if passed == total:
        print("[SUCCESS] All validations passed! The project is ready for quickstart.")
        return True
    else:
        print(f"[ERROR] {total - passed} checks failed. Please address the issues above.")
        return False


if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)