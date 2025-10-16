#!/usr/bin/env python3
"""
Simple test runner for the validator.
Runs 3 test cases and reports results.
"""

from pathlib import Path
import sys

# Add parent directory to path so we can import validator
sys.path.insert(0, str(Path(__file__).parent.parent))

from validator.validate import validate

def run_tests():
    """Run all test cases and report results."""
    
    tests_dir = Path(__file__).parent
    
    test_cases = [
        ("valid_robot_script.py", True, "Valid robot script"),
        ("invalid_robot_script.py", False, "Invalid API method (fly_to_moon)"),
        ("invalid_nonsense.py", False, "Forbidden operations (print, eval, os)"),
    ]
    
    print("=" * 60)
    print("Running Validator Tests")
    print("=" * 60)
    
    passed = 0
    failed = 0
    
    for filename, expected, description in test_cases:
        test_file = tests_dir / filename
        
        if not test_file.exists():
            print(f"[SKIP] {filename} (file not found)")
            continue
        
        result = validate(test_file)
        
        if result == expected:
            print(f"[PASS] {filename}")
            print(f"       {description}")
            print(f"       Expected: {expected}, Got: {result}")
            passed += 1
        else:
            print(f"[FAIL] {filename}")
            print(f"       {description}")
            print(f"       Expected: {expected}, Got: {result}")
            failed += 1
        
        print()
    
    print("=" * 60)
    print(f"Results: {passed} passed, {failed} failed")
    print("=" * 60)
    
    return failed == 0

if __name__ == "__main__":
    success = run_tests()
    sys.exit(0 if success else 1)

