from pathlib import Path
import sys
import time
import re

from generator.llm_client import generate_policy
from validator.validate import validate

BASE = Path(__file__).parent
POLICY_DIR = BASE / "policies"
POLICY_DIR.mkdir(exist_ok=True)

def slugify(text: str) -> str:
    s = re.sub(r"[^a-zA-Z0-9]+", "-", text.strip().lower()).strip("-")
    return s or "command"

def save_policy(code: str, command: str) -> Path:
    ts = time.strftime("%Y%m%d-%H%M%S")
    name = f"{ts}_{slugify(command)}.py"
    path = POLICY_DIR / name
    # Force LF endings for portability
    with open(path, "w", encoding="utf-8", newline="\n") as f:
        f.write(code)
    return path

def main():
    if len(sys.argv) >= 2:
        nl_command = " ".join(sys.argv[1:])
    else:
        nl_command = input("Enter command: ").strip()

    code = generate_policy(nl_command)
    
    # Check if LLM determined request is invalid for robot
    if code.strip() == "INVALID":
        print("INVALID --> Command is invalid for robot -- False")
        return
    
    # Otherwise, save and validate the generated policy
    policy_path = save_policy(code, nl_command)
    is_valid = validate(policy_path)
    # Print ONLY True or False
    print("True --> Would proceed to robot execution" if is_valid else "False --> Code fails validation and will not be executed")

if __name__ == "__main__":
    main()

