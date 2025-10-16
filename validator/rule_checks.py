import ast

ALLOWED_ATTRS = {
    "set_cartesian_target",
    "set_joint_targets",
    "plan",
    "execute",
    "open_gripper",
    "close_gripper",
    "sleep",
}

def _only_api_calls(tree: ast.AST) -> bool:
    """
    Enforce that all Attribute calls are api.<allowed>.
    Also forbid Name-based calls except local vars and literals (no bare calls).
    """
    allowed = ALLOWED_ATTRS
    for node in ast.walk(tree):
        if isinstance(node, ast.Call):
            f = node.func
            # api.method(...)
            if isinstance(f, ast.Attribute) and isinstance(f.value, ast.Name) and f.value.id == "api":
                if f.attr not in allowed:
                    return False
            # Disallow bare Name(...) calls (e.g., foo()), except if it's a local variable
            elif isinstance(f, ast.Name):
                # We conservatively forbid Name calls to avoid sneaky builtins
                return False
    return True

def _plan_before_execute(tree: ast.AST) -> bool:
    """
    Cheap static check: require that execute(plan) is called with a Name 'plan'
    that was previously assigned from api.plan(...).
    """
    plan_defined_lines = set()
    execute_lines = []

    for node in ast.walk(tree):
        if isinstance(node, ast.Assign):
            # plan = api.plan(...)
            if len(node.targets) == 1 and isinstance(node.targets[0], ast.Name) and node.targets[0].id == "plan":
                if isinstance(node.value, ast.Call) and isinstance(node.value.func, ast.Attribute):
                    f = node.value.func
                    if isinstance(f.value, ast.Name) and f.value.id == "api" and f.attr == "plan":
                        plan_defined_lines.add(node.lineno)

        if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute):
            f = node.func
            if isinstance(f.value, ast.Name) and f.value.id == "api" and f.attr == "execute":
                # must be execute(plan)
                if len(node.args) != 1 or not isinstance(node.args[0], ast.Name) or node.args[0].id != "plan":
                    return False
                execute_lines.append(node.lineno)

    if execute_lines and not plan_defined_lines:
        return False
    # Ensure at least one plan is earlier than the earliest execute
    return (not execute_lines) or (min(plan_defined_lines) < min(execute_lines))

def rules_ok(code: str) -> bool:
    try:
        tree = ast.parse(code, mode="exec")
    except Exception:
        return False
    return _only_api_calls(tree) and _plan_before_execute(tree)

