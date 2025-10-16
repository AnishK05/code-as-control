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

BANNED_CALL_NAMES = {"print", "open", "input", "exec", "eval", "__import__", "compile"}

# Allowed imports
ALLOWED_IMPORT_MODULES = {
    "rospy",
    "geometry_msgs.msg",
}

# Allowed imports from geometry_msgs.msg
ALLOWED_GEOMETRY_IMPORTS = {
    "Pose", "Point", "Quaternion", "PoseStamped"
}

ALLOWED_NODES = {
    ast.Module, ast.FunctionDef, ast.arguments, ast.arg,
    ast.Expr, ast.Call, ast.Assign, ast.AnnAssign, ast.Return, ast.Pass,
    ast.Name, ast.Load, ast.Store, ast.Attribute, ast.Constant,
    ast.Tuple, ast.List, ast.Dict,
    ast.If, ast.Compare, ast.BoolOp, ast.UnaryOp, ast.BinOp,
    ast.For,  # allow simple loops if the model emits them
    ast.While, # optional; keep if you want
    ast.keyword,  # for keyword arguments like vel=0.15
    ast.USub, ast.UAdd,  # unary operators for negative/positive numbers
    ast.Add, ast.Sub, ast.Mult, ast.Div, ast.FloorDiv, ast.Mod, ast.Pow,  # binary operators
    ast.Eq, ast.NotEq, ast.Lt, ast.LtE, ast.Gt, ast.GtE,  # comparison operators
    ast.And, ast.Or, ast.Not,  # boolean operators
    ast.Import, ast.ImportFrom, ast.alias,  # for allowed imports (validated separately)
}

BANNED_NODES = {
    ast.Global, ast.Nonlocal, ast.Lambda,
    ast.Try, ast.Raise, ast.ClassDef, ast.Delete, ast.With, ast.Await, ast.AsyncFunctionDef,
    ast.ListComp, ast.DictComp, ast.SetComp, ast.GeneratorExp,
}
# Note: Import and ImportFrom are now allowed but validated separately

def _single_run_signature_ok(tree: ast.Module) -> bool:
    funcs = [n for n in tree.body if isinstance(n, ast.FunctionDef)]
    
    # Allow imports at top level (geometry_msgs, rospy)
    imports = [n for n in tree.body if isinstance(n, (ast.Import, ast.ImportFrom))]
    
    # Only function and imports allowed at top level
    if len(funcs) + len(imports) != len(tree.body):
        return False
    
    if len(funcs) != 1:
        return False
    
    f = funcs[0]
    if f.name != "run":
        return False
    # exactly one arg named 'group'
    if not isinstance(f.args, ast.arguments):
        return False
    args = f.args.args
    if len(args) != 1 or args[0].arg != "group":
        return False
    
    return True

def _node_whitelist_ok(tree: ast.AST) -> bool:
    for node in ast.walk(tree):
        if type(node) in BANNED_NODES:
            return False
        if type(node) not in ALLOWED_NODES:
            return False
    return True

def _no_banned_calls(tree: ast.AST) -> bool:
    for node in ast.walk(tree):
        if isinstance(node, ast.Call):
            # Name(...) calls
            if isinstance(node.func, ast.Name) and node.func.id in BANNED_CALL_NAMES:
                return False
    return True

def _no_dunders_or_builtins_attr(tree: ast.AST) -> bool:
    for node in ast.walk(tree):
        if isinstance(node, ast.Attribute):
            # Disallow dunder attributes on any object
            if node.attr.startswith("__"):
                return False
        if isinstance(node, ast.Name):
            if node.id.startswith("__") and node.id != "__name__":
                return False
    return True

def _valid_imports_only(tree: ast.Module) -> bool:
    """Check that only allowed imports are used"""
    for node in tree.body:
        if isinstance(node, ast.Import):
            # e.g., import rospy
            for alias in node.names:
                if alias.name not in ALLOWED_IMPORT_MODULES:
                    return False
        elif isinstance(node, ast.ImportFrom):
            # e.g., from geometry_msgs.msg import Pose
            if node.module not in ALLOWED_IMPORT_MODULES:
                return False
            # Check what's being imported from geometry_msgs.msg
            if node.module == "geometry_msgs.msg":
                for alias in node.names:
                    if alias.name not in ALLOWED_GEOMETRY_IMPORTS:
                        return False
    return True

def ast_safe(code: str) -> bool:
    try:
        tree = ast.parse(code, mode="exec")
    except Exception:
        return False
    if not _single_run_signature_ok(tree):
        return False
    if not _valid_imports_only(tree):
        return False
    if not _node_whitelist_ok(tree):
        return False
    if not _no_banned_calls(tree):
        return False
    if not _no_dunders_or_builtins_attr(tree):
        return False
    # Basic size limit
    if code.count("\n") > 200:
        return False
    return True

