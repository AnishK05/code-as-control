import os
import re
from typing import List, Dict, Tuple
from pathlib import Path
from dotenv import load_dotenv
from .prompts import SYSTEM_PROMPT, USER_PROMPT_TEMPLATE, ERROR_FIXING_PROMPT

load_dotenv()

API_KEY = os.getenv("GEMINI_API_KEY")
MODEL = os.getenv("GEMINI_MODEL", "gemini-3-pro-preview")
# Thinking level: "low" (fast, simple tasks) or "high" (deep reasoning, complex tasks)
THINKING_LEVEL = os.getenv("GEMINI_THINKING_LEVEL", "high")

# Path to the Sawyer robot initial state image
GENERATOR_DIR = Path(__file__).parent
SAWYER_IMAGE_PATH = GENERATOR_DIR / "SawyerInitialState.png"

def _load_sawyer_image():
    """Load the Sawyer robot image for visual context."""
    try:
        from google import genai
        from google.genai import types
    except ImportError:
        return None
    
    if not SAWYER_IMAGE_PATH.exists():
        return None
    
    try:
        # Upload the image to Gemini
        uploaded_file = genai.upload_file(path=str(SAWYER_IMAGE_PATH))
        return uploaded_file
    except Exception:
        # If upload fails, try reading as bytes
        try:
            with open(SAWYER_IMAGE_PATH, 'rb') as f:
                image_bytes = f.read()
            return types.Part.from_bytes(data=image_bytes, mime_type='image/png')
        except Exception:
            return None


def _call_gemini(messages: List[Dict[str, str]], include_image: bool = True) -> str:
    """
    Internal function to call Gemini API with a list of messages.
    
    Args:
        messages: List of message dicts with 'role' and 'content' keys
        include_image: Whether to include the Sawyer robot image (default True)
        
    Returns:
        The text response from Gemini
    """
    if not API_KEY:
        raise ValueError("GEMINI_API_KEY not found in environment. Please set it in your .env file.")
    
    try:
        from google import genai
        from google.genai import types
    except ImportError as e:
        raise ImportError(
            "Google GenAI library not found. Install it with: pip install google-generativeai"
        ) from e
    
    # Create client with API key from environment
    client = genai.Client(api_key=API_KEY)
    
    # Load the Sawyer image for visual context
    sawyer_image = None
    if include_image:
        sawyer_image = _load_sawyer_image()
    
    # Build contents for multi-turn conversation
    # First message should include system prompt and optionally the robot image
    contents = []
    for i, msg in enumerate(messages):
        if i == 0 and msg['role'] == 'user':
            # Prepend system prompt to first user message
            text_content = SYSTEM_PROMPT + "\n\n" + msg['content']
            
            # If we have the image, send it with the first message for visual context
            if sawyer_image is not None:
                # Create multimodal content with image + text
                content = [sawyer_image, text_content]
            else:
                content = text_content
        else:
            content = msg['content']
        
        contents.append(content)
    
    try:
        # Call Gemini 3 API
        # Temperature defaults to 1.0 (optimal for Gemini 3 reasoning)
        # thinking_level: "low" for fast responses, "high" for deep reasoning
        from google.genai import types
        
        response = client.models.generate_content(
            model=MODEL,
            contents=contents,
        )
        
        # Get the text response - try different methods
        if hasattr(response, 'text'):
            text = response.text
        elif hasattr(response, 'candidates') and response.candidates:
            # Try accessing via candidates
            text = response.candidates[0].content.parts[0].text
        else:
            # Last resort: convert to string
            text = str(response)
        
    except Exception as e:
        raise RuntimeError(f"Gemini API call failed: {str(e)}") from e
    
    if not text or not text.strip():
        raise ValueError(f"Gemini returned empty response. Full response: {response}")
    
    return text

def generate_policy_interactive(nl_command: str, conversation_history: List[Dict[str, str]] = None) -> Tuple[str, str]:
    """
    Generate a policy using Google Gemini API with conversational support.
    
    Args:
        nl_command: The natural language command
        conversation_history: Optional list of previous messages in conversation
        
    Returns:
        Tuple of (response_type, content) where:
        - response_type is "QUESTION", "INVALID", or "CODE"
        - content is the question text or generated code
    """
    if conversation_history is None:
        conversation_history = []
    
    # If this is the first message, add it to history
    if not conversation_history:
        conversation_history.append({
            'role': 'user',
            'content': USER_PROMPT_TEMPLATE.format(command=nl_command)
        })
    
    text = _call_gemini(conversation_history)
    
    # Check if Gemini determined the request is invalid for a robot
    if text.strip().upper() == "INVALID":
        return ("INVALID", "INVALID")
    
    # Check if Gemini is asking a clarifying question
    if text.strip().upper().startswith("QUESTION:"):
        # Remove the QUESTION: prefix and return the question text
        question_text = text.strip()[9:].strip()  # Remove "QUESTION:" prefix
        return ("QUESTION", question_text)
    
    # Otherwise, try to extract code
    # Extract code from markdown fences if present
    m = re.search(r"```(?:python)?\s*\n([\s\S]*?)```", text, re.IGNORECASE)
    if m:
        code = m.group(1)
        return ("CODE", code.strip() + "\n")
    
    # If no fences but looks like code (contains "def run"), return as code
    if "def run" in text:
        return ("CODE", text.strip() + "\n")
    
    # Otherwise, treat it as a question or clarification
    return ("QUESTION", text.strip())

def generate_policy(nl_command: str) -> str:
    """
    Generate a policy using Google Gemini API (legacy single-call version).
    Returns the generated Python code as a string.
    
    Note: This is kept for backwards compatibility. For interactive conversations,
    use generate_policy_interactive() instead.
    """
    response_type, content = generate_policy_interactive(nl_command)
    
    if response_type == "INVALID":
        return "INVALID"
    elif response_type == "CODE":
        return content
    else:
        # If it's a question on first call, just generate without interaction
        # This shouldn't happen in practice with the new CLI flow
        return content

def fix_policy_with_errors(original_code: str, error_log: str, command: str) -> str:
    """
    Fix a policy that encountered errors during execution.
    
    Args:
        original_code: The original policy code that failed
        error_log: The error log from execution
        command: The original command the policy was meant to accomplish
        
    Returns:
        The fixed policy code as a string
    """
    # Build the error fixing prompt
    prompt = ERROR_FIXING_PROMPT.format(
        command=command,
        original_code=original_code,
        error_log=error_log
    )
    
    # Create a single-message conversation for error fixing
    messages = [{'role': 'user', 'content': prompt}]
    
    text = _call_gemini(messages)
    
    # Extract code from markdown fences if present
    m = re.search(r"```(?:python)?\s*\n([\s\S]*?)```", text, re.IGNORECASE)
    if m:
        code = m.group(1)
        return code.strip() + "\n"
    
    # If no fences, return the raw text
    return text.strip() + "\n"