import os
import re
from dotenv import load_dotenv
from .prompts import SYSTEM_PROMPT, USER_PROMPT_TEMPLATE

load_dotenv()

API_KEY = os.getenv("GEMINI_API_KEY")
MODEL = os.getenv("GEMINI_MODEL", "gemini-2.5-flash")

def generate_policy(nl_command: str) -> str:
    """
    Generate a policy using Google Gemini API.
    Returns the generated Python code as a string.
    """
    if not API_KEY:
        raise ValueError("GEMINI_API_KEY not found in environment. Please set it in your .env file.")
    
    try:
        from google import genai
    except ImportError as e:
        raise ImportError(
            "Google GenAI library not found. Install it with: pip install google-generativeai"
        ) from e
    
    # Create client with API key from environment
    client = genai.Client(api_key=API_KEY)
    
    # Build the full prompt
    prompt = SYSTEM_PROMPT + "\n\n" + USER_PROMPT_TEMPLATE.format(command=nl_command)
    
    try:
        # Call Gemini API using the new SDK
        response = client.models.generate_content(
            model=MODEL,
            contents=prompt,
            config={
                "temperature": 0.2,
                "max_output_tokens": 512,
                "thinking_config": {
                    "thinking_budget": 0  # Disable thinking to avoid token waste
                }
            }
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

    # Check if Gemini determined the request is invalid for a robot
    if text.strip().upper() == "INVALID":
        return "INVALID"

    # Extract code from markdown fences if present
    m = re.search(r"```(?:python)?\s*\n([\s\S]*?)```", text, re.IGNORECASE)
    if m:
        code = m.group(1)
        return code.strip() + "\n"
    
    # If no fences, return the raw text
    return text.strip() + "\n"