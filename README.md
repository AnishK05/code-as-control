# Code as Control: LLM-Generated Robot Policies from Natural Language

A novel paradigm where LLMs generate executable Python code directly from natural language to control robot arms.

**Paper:** [FinalPaper_CodeAsControl.pdf](FinalPaper_CodeAsControl.pdf)  
**Videos & Demos:** [Google Drive](https://drive.google.com/drive/folders/1v80Uo4iHuvqPK38lfNLo67hZSUBitxhq?usp=drive_link)

## Overview

Traditional robot interfaces use low-level commands or predefined behaviors. This research explores **executable code as the control interface**: users give natural language commands (e.g., "wave to me"), and the LLM generates safe Python policies that execute directly on a Sawyer robot via ROS/MoveIt.

## Novel Contributions

**1. Code-as-Control Paradigm**  
The system generates complete executable Python code, not parameters or API calls. Natural language → validated Python policy → direct robot execution.

**2. Interactive Clarification**  
The LLM proactively asks clarifying questions about speed, timing, amplitude, and style before generating code, ensuring generated motions match human intent.

**3. Safety Validation Pipeline**  
Multi-layer validation (AST analysis + rule checking) prevents execution of unsafe code. Validates imports, detects forbidden operations, and enforces workspace constraints.

**4. Error-Driven Refinement**  
Failed executions generate detailed error logs that the LLM analyzes to automatically fix and regenerate policies.

## Technical Details

**Generated Policy Example:**
```python
from geometry_msgs.msg import Pose, Point, Quaternion
import rospy

def run(group):
    pose = Pose()
    pose.position = Point(0.6, 0.2, 0.4)
    pose.orientation = Quaternion(0, 0, 0, 1)
    
    group.set_pose_target(pose)
    group.go(wait=True)
    group.stop()
    group.clear_pose_targets()
    rospy.sleep(1.0)
```

**Validation:** AST safety checks + rule-based validation ensure policies only use approved operations and stay within workspace bounds.

## Running

```bash
# Generate policy
python cli.py

# Execute on robot (requires ROS + MoveIt)
python3 execute_policy.py policies/TIMESTAMP.py
```

## System Architecture

```
User Command → LLM (Clarifying Questions) → Code Generator → Validator → Robot Execution
```

The LLM uses Google Gemini, policies execute on a Sawyer 7-DOF robot arm via ROS/MoveIt.

## Citation

```bibtex
@article{kalra2025codeAsControl,
  title={Code as Control: LLM-Generated Robot Policies from Natural Language},
  author={Kalra, Anish},
  year={2025}
}
```
---

**License:** Academic and research use  
⚠️ Research prototype - always supervise robot execution
