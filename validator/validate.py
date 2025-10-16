from .ast_checks import ast_safe
from .rule_checks import rules_ok

def validate(policy_path) -> bool:
    try:
        with open(policy_path, "r", encoding="utf-8") as f:
            code = f.read()
        return ast_safe(code) and rules_ok(code)
    except Exception:
        return False

