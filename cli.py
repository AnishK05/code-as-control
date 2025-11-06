from pathlib import Path
import sys
import time
import re
from typing import Optional, Tuple

from generator.llm_client import generate_policy_interactive, fix_policy_with_errors
from validator.validate import validate

BASE = Path(__file__).parent
POLICY_DIR = BASE / "policies"
POLICY_DIR.mkdir(exist_ok=True)
ERROR_LOG_DIR = BASE / "error_logs"
ERROR_LOG_DIR.mkdir(exist_ok=True)

def slugify(text: str) -> str:
    s = re.sub(r"[^a-zA-Z0-9]+", "-", text.strip().lower()).strip("-")
    return s or "command"


def select_mode() -> int:
    """
    Display mode selection menu and get user choice.
    
    Returns:
        1 for generate new policy, 2 for edit/fix existing policy
    """
    print("\n" + "=" * 60)
    print("Robot Policy Generation System")
    print("=" * 60)
    print("\nWhat would you like to do?")
    print("  1. Generate a new policy")
    print("  2. Edit/fix an existing policy")
    print()
    
    while True:
        choice = input("Enter your choice (1 or 2): ").strip()
        if choice in ['1', '2']:
            return int(choice)
        print("Invalid choice. Please enter 1 or 2.")


def select_policy_for_editing() -> Tuple[Optional[Path], Optional[str]]:
    """
    Let user select a policy to edit, either from a list or by manual path entry.
    
    Returns:
        Tuple of (policy_path, original_command) or (None, None) if cancelled
    """
    print("\n" + "-" * 60)
    print("Select a policy to edit:")
    print("-" * 60)
    
    # Get all policy files, sorted by modification time (most recent first)
    policy_files = sorted(POLICY_DIR.glob("*.py"), key=lambda p: p.stat().st_mtime, reverse=True)
    
    if not policy_files:
        print("No policies found in the policies directory.")
        print("Please enter a manual path or press Enter to cancel.")
        manual_path = input("Policy path: ").strip()
        if not manual_path:
            return None, None
        
        policy_path = Path(manual_path)
        if not policy_path.exists():
            print(f"Error: File not found: {policy_path}")
            return None, None
        
        # Extract command from filename if possible (won't always work)
        command = "unknown command"
        return policy_path, command
    
    # Display numbered list of policies
    print("\nAvailable policies (most recent first):")
    for i, policy_file in enumerate(policy_files, 1):
        # Show relative path and file size
        size = policy_file.stat().st_size
        mod_time = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(policy_file.stat().st_mtime))
        print(f"  {i}. {policy_file.name}")
        print(f"     Modified: {mod_time}, Size: {size} bytes")
    
    print("\nOptions:")
    print("  - Enter a number to select a policy")
    print("  - Enter a file path to load a policy manually")
    print("  - Press Enter to cancel")
    print()
    
    choice = input("Your choice: ").strip()
    
    if not choice:
        return None, None
    
    # Check if it's a number
    if choice.isdigit():
        idx = int(choice) - 1
        if 0 <= idx < len(policy_files):
            policy_path = policy_files[idx]
            # Try to extract command from filename (best effort)
            command = "unknown command"
            return policy_path, command
        else:
            print(f"Invalid selection: {choice}")
            return None, None
    else:
        # Treat as manual path
        policy_path = Path(choice)
        if not policy_path.exists():
            print(f"Error: File not found: {policy_path}")
            return None, None
        
        command = "unknown command"
        return policy_path, command


def select_error_log() -> Optional[str]:
    """
    Let user select or enter an error log file to use for fixing.
    
    Returns:
        Path to error log file as string, or None if cancelled
    """
    print("\n" + "-" * 60)
    print("Select an error log (optional):")
    print("-" * 60)
    
    # Get all error log files, sorted by modification time (most recent first)
    error_files = sorted(ERROR_LOG_DIR.glob("*.txt"), key=lambda p: p.stat().st_mtime, reverse=True)
    
    if not error_files:
        print("No error logs found. You can enter a manual path or press Enter to skip.")
        manual_path = input("Error log path (or Enter to skip): ").strip()
        return manual_path if manual_path else None
    
    # Display numbered list of error logs
    print("\nRecent error logs:")
    for i, error_file in enumerate(error_files[:10], 1):  # Show only last 10
        mod_time = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(error_file.stat().st_mtime))
        print(f"  {i}. {error_file.name}")
        print(f"     Created: {mod_time}")
    
    print("\nOptions:")
    print("  - Enter a number to select an error log")
    print("  - Enter a file path to load an error log manually")
    print("  - Press Enter to skip (provide error details manually)")
    print()
    
    choice = input("Your choice: ").strip()
    
    if not choice:
        return None
    
    # Check if it's a number
    if choice.isdigit():
        idx = int(choice) - 1
        if 0 <= idx < len(error_files):
            return str(error_files[idx])
        else:
            print(f"Invalid selection: {choice}")
            return None
    else:
        # Treat as manual path
        if Path(choice).exists():
            return choice
        else:
            print(f"Error: File not found: {choice}")
            return None


def generate_new_policy_interactive() -> Tuple[Optional[str], Optional[str]]:
    """
    Interactive policy generation with conversational follow-up questions.
    
    Returns:
        Tuple of (generated_code, command) or (None, None) if cancelled/failed
    """
    print("\n" + "-" * 60)
    print("Generate New Policy")
    print("-" * 60)
    
    # Get initial command
    nl_command = input("\nEnter robot command: ").strip()
    if not nl_command:
        print("No command provided. Cancelled.")
        return None, None
    
    # Start conversation history
    conversation_history = []
    
    while True:
        print("\n[Contacting Gemini...]")
        
        try:
            response_type, content = generate_policy_interactive(nl_command, conversation_history)
        except Exception as e:
            print(f"\nError: {e}")
            return None, None
        
        if response_type == "INVALID":
            print("\n" + "!" * 60)
            print("INVALID COMMAND")
            print("!" * 60)
            print("The command is not appropriate for a robot arm.")
            print("Please provide a command related to physical manipulation tasks.")
            return None, None
        
        elif response_type == "QUESTION":
            # Gemini is asking clarifying questions
            print("\n" + "-" * 60)
            print("Gemini has some questions:")
            print("-" * 60)
            print(content)
            print()
            
            # Get user's answer
            user_answer = input("Your answer: ").strip()
            if not user_answer:
                print("No answer provided. Cancelling.")
                return None, None
            
            # Add assistant's question and user's answer to history
            conversation_history.append({'role': 'assistant', 'content': f"QUESTION:\n{content}"})
            conversation_history.append({'role': 'user', 'content': user_answer})
            
            # Continue the loop to get next response
            continue
        
        elif response_type == "CODE":
            # Gemini generated the code
            print("\n[Code generated successfully!]")
            return content, nl_command
        
        else:
            print(f"\nUnexpected response type: {response_type}")
            return None, None


def save_policy(code: str, command: str) -> Path:
    ts = time.strftime("%Y%m%d-%H%M%S")
    name = f"{ts}.py"
    path = POLICY_DIR / name
    # Force LF endings for portability
    with open(path, "w", encoding="utf-8", newline="\n") as f:
        f.write(code)
    return path

def handle_generate_mode():
    """Handle the 'generate new policy' mode."""
    code, nl_command = generate_new_policy_interactive()
    
    if code is None:
        # User cancelled or error occurred
        return
    
    # Save and validate the generated policy
    policy_path = save_policy(code, nl_command)
    print(f"\n[Policy saved to: {policy_path}]")
    
    print("\n[Validating policy...]")
    is_valid = validate(policy_path)
    
    # Print validation result
    print("\n" + "=" * 60)
    if is_valid:
        print("✓ VALIDATION PASSED")
        print("=" * 60)
        print(f"Policy is ready for execution: {policy_path}")
        print("\nTo execute on robot:")
        print(f"  python3 execute_policy.py {policy_path}")
    else:
        print("✗ VALIDATION FAILED")
        print("=" * 60)
        print("The generated code does not pass validation checks.")
        print("Please review the code or try regenerating with more specific instructions.")


def handle_edit_mode():
    """Handle the 'edit/fix existing policy' mode."""
    # Select policy to edit
    policy_path, original_command = select_policy_for_editing()
    
    if policy_path is None:
        print("No policy selected. Cancelled.")
        return
    
    print(f"\n[Selected policy: {policy_path}]")
    
    # Read the original policy code
    try:
        with open(policy_path, "r", encoding="utf-8") as f:
            original_code = f.read()
    except Exception as e:
        print(f"Error reading policy file: {e}")
        return
    
    # Ask user for the original command if not extracted
    if original_command == "unknown command":
        print("\nWhat was the original command for this policy?")
        original_command = input("Original command: ").strip()
        if not original_command:
            original_command = "move the robot arm"  # Default fallback
    
    # Select or provide error log
    error_log_path = select_error_log()
    
    error_log_content = ""
    if error_log_path:
        try:
            with open(error_log_path, "r", encoding="utf-8") as f:
                error_log_content = f.read()
            print(f"\n[Using error log: {error_log_path}]")
        except Exception as e:
            print(f"Warning: Could not read error log: {e}")
            error_log_content = ""
    
    # If no error log, ask user to describe the problem
    if not error_log_content:
        print("\nNo error log provided. Please describe the issue with the policy:")
        user_description = input("Issue description: ").strip()
        if not user_description:
            print("No issue description provided. Cancelled.")
            return
        error_log_content = f"User-reported issue: {user_description}"
    
    # Call Gemini to fix the policy
    print("\n[Contacting Gemini to fix the policy...]")
    try:
        fixed_code = fix_policy_with_errors(original_code, error_log_content, original_command)
    except Exception as e:
        print(f"\nError: {e}")
        return
    
    print("\n[Fixed code generated successfully!]")
    
    # Save the fixed policy
    policy_path = save_policy(fixed_code, original_command + " (fixed)")
    print(f"\n[Fixed policy saved to: {policy_path}]")
    
    print("\n[Validating fixed policy...]")
    is_valid = validate(policy_path)
    
    # Print validation result
    print("\n" + "=" * 60)
    if is_valid:
        print("✓ VALIDATION PASSED")
        print("=" * 60)
        print(f"Fixed policy is ready for execution: {policy_path}")
        print("\nTo execute on robot:")
        print(f"  python3 execute_policy.py {policy_path}")
    else:
        print("✗ VALIDATION FAILED")
        print("=" * 60)
        print("The fixed code does not pass validation checks.")
        print("You may need to provide more details about the error.")


def main():
    """Main entry point for the CLI."""
    # Check if command-line arguments provided (legacy mode for backwards compatibility)
    if len(sys.argv) >= 2:
        # Legacy mode: quick generation without interactive mode
        print("[Running in legacy mode - generating policy directly]")
        nl_command = " ".join(sys.argv[1:])
        
        from generator.llm_client import generate_policy
        code = generate_policy(nl_command)
        
        # Check if LLM determined request is invalid for robot
        if code.strip() == "INVALID":
            print("INVALID --> Command is invalid for robot -- False")
            return
        
        # Otherwise, save and validate the generated policy
        policy_path = save_policy(code, nl_command)
        is_valid = validate(policy_path)
        
        # Print validation result
        print("True --> Would proceed to robot execution" if is_valid else "False --> Code fails validation and will not be executed")
        
        # If valid, indicate we would execute
        if is_valid:
            print(f"Executing code... (saved to {policy_path})")
        return
    
    # Interactive mode
    mode = select_mode()
    
    if mode == 1:
        handle_generate_mode()
    elif mode == 2:
        handle_edit_mode()
    else:
        print("Invalid mode selected.")
        return

if __name__ == "__main__":
    main()

