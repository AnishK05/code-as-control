# Code as Control: LLM-Generated Robot Policies from Natural Language

This repository contains the implementation and research artifacts for **"Code as Control"** - a novel paradigm for robot control that leverages Large Language Models (LLMs) to generate executable Python policies directly from natural language commands.

## Overview

Traditional robot control interfaces require either low-level commands, predefined behaviors, or complex parameter specifications. This research explores a fundamentally different approach: **treating executable code itself as the control interface**. By leveraging LLMs' code generation capabilities, we enable natural language instructions to be translated directly into safe, executable robot motion policies.

## Key Innovation

Instead of generating parameters or commands, our system:
1. **Translates natural language** (e.g., "wave to me", "draw a figure eight") **→ executable Python code**
2. **Validates safety** through AST-based static analysis and rule checking
3. **Executes policies** on a Sawyer robot arm using ROS/MoveIt
4. **Iteratively improves** policies using error feedback loops

This "code as control" paradigm combines the expressiveness of natural language with the precision and safety of programmatic control.

## Research Contributions

### 1. **Natural Language to Robot Code Translation**
- Direct generation of MoveIt-compatible Python policies from user commands
- No intermediate representations or parameter mappings required
- Leverages Google Gemini's code generation and reasoning capabilities

### 2. **Interactive Clarification System**
- LLM proactively asks clarifying questions before generating code
- Captures nuanced details about speed, timing, range, smoothness, and style
- Ensures generated motions match human intent

### 3. **Safety-First Validation**
- Multi-layer validation pipeline prevents unsafe code execution
- AST-based static analysis detects dangerous operations
- Rule-based checks enforce workspace constraints and allowed operations
- Policies are validated before execution on physical hardware

### 4. **Error-Aware Iterative Refinement**
- Automatic error logging during policy execution
- LLM-powered error analysis and policy fixing
- Users can iterate on failed policies with targeted corrections

## System Architecture

```
┌─────────────────┐
│  User Command   │  "Wave to me with a smooth, gentle motion"
└────────┬────────┘
         │
         v
┌─────────────────┐
│  LLM (Gemini)   │  Asks: "How many waves? What speed? What amplitude?"
└────────┬────────┘
         │
         v
┌─────────────────┐
│  Code Generator │  Generates Python policy with MoveIt commands
└────────┬────────┘
         │
         v
┌─────────────────┐
│  Validator      │  AST checks + Rule validation
└────────┬────────┘
         │
         v
┌─────────────────┐
│  ROS Executor   │  Executes on Sawyer robot via MoveIt
└─────────────────┘
```

## Technical Implementation

### Generated Policy Format

Each policy is a self-contained Python module with a `run(group)` function:

```python
from geometry_msgs.msg import Pose, Point, Quaternion
import rospy

def run(group):
    # Move to position
    pose = Pose()
    pose.position = Point(0.6, 0.2, 0.4)
    pose.orientation = Quaternion(0, 0, 0, 1)
    
    group.set_pose_target(pose)
    group.go(wait=True)
    group.stop()
    group.clear_pose_targets()
    
    rospy.sleep(1.0)
```

### Validation Pipeline

1. **AST Safety Checks** (`validator/ast_checks.py`)
   - Whitelist-based import validation
   - Forbidden function detection (eval, exec, open, etc.)
   - Malicious code pattern detection

2. **Rule-Based Checks** (`validator/rule_checks.py`)
   - Proper function signature validation
   - Required imports verification
   - Workspace coordinate constraints

## Citation

If you use this work in your research, please cite:

```bibtex
@article{codeAsControl2025,
  title={Code as Control: LLM-Generated Robot Policies from Natural Language},
  author={[Anish Kalra]},
  year={2025}
}
```

## License

This project is released for academic and research purposes.

**Note:** This is a research prototype. Always supervise robot execution and maintain safety protocols when working with physical hardware.
