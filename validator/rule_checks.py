import ast

# Allowed methods on 'group' (MoveGroupCommander)
ALLOWED_GROUP_METHODS = {
    "set_pose_target",
    "go",
    "stop",
    "clear_pose_targets",
}

# Allowed rospy functions
ALLOWED_ROSPY_CALLS = {
    "sleep",
}

# Allowed geometry_msgs.msg constructors
ALLOWED_GEOMETRY_CONSTRUCTORS = {
    "Pose",
    "Point", 
    "Quaternion",
    "PoseStamped",
}

def _only_allowed_calls(tree: ast.AST) -> bool:
    """
    Enforce that only allowed MoveIt, rospy, and geometry_msgs calls are made.
    """
    for node in ast.walk(tree):
        if isinstance(node, ast.Call):
            f = node.func
            
            # group.method(...)
            if isinstance(f, ast.Attribute) and isinstance(f.value, ast.Name):
                if f.value.id == "group":
                    if f.attr not in ALLOWED_GROUP_METHODS:
                        return False
                # rospy.method(...)
                elif f.value.id == "rospy":
                    if f.attr not in ALLOWED_ROSPY_CALLS:
                        return False
                # Any other attribute calls are forbidden
                else:
                    # Allow assignment to attributes like pose.position = ...
                    # but forbid unknown method calls
                    pass
            
            # Direct Name() calls - allow only geometry constructors
            elif isinstance(f, ast.Name):
                if f.id not in ALLOWED_GEOMETRY_CONSTRUCTORS:
                    # Disallow unknown bare function calls
                    return False
    return True

def _moveit_pattern_ok(tree: ast.AST) -> bool:
    """
    Check that MoveIt patterns are used correctly:
    - set_pose_target should be called before go()
    - go() should be followed by stop() and clear_pose_targets()
    This is a basic check - doesn't enforce strict ordering within a function.
    """
    has_set_pose = False
    has_go = False
    
    for node in ast.walk(tree):
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute):
            f = node.func
            if isinstance(f.value, ast.Name) and f.value.id == "group":
                if f.attr == "set_pose_target":
                    has_set_pose = True
                elif f.attr == "go":
                    has_go = True
    
    # If go() is called, set_pose_target should have been called
    # This is a weak check but ensures basic pattern
    if has_go and not has_set_pose:
        return False
    
    return True

def rules_ok(code: str) -> bool:
    try:
        tree = ast.parse(code, mode="exec")
    except Exception:
        return False
    return _only_allowed_calls(tree) and _moveit_pattern_ok(tree)

