#!/usr/bin/env python3
"""
ROS node to execute generated policies on the Sawyer robot.

Usage:
    roslaunch sawyer_moveit_config sim_sawyer_moveit.launch electric_gripper:=true
    
    # In another terminal:
    python3 execute_policy.py policies/20251016-135314_wave-to-me.py
"""

import rospy
import sys
import moveit_commander
from pathlib import Path
import time
import traceback
import io
import contextlib

# Create error logs directory
BASE = Path(__file__).parent
ERROR_LOG_DIR = BASE / "error_logs"
ERROR_LOG_DIR.mkdir(exist_ok=True)


def create_error_log(policy_path: Path, error_message: str) -> Path:
    """
    Create an error log file for a failed policy execution.
    
    Args:
        policy_path: Path to the policy file that failed
        error_message: The full error message/traceback
        
    Returns:
        Path to the created error log file
    """
    ts = time.strftime("%Y%m%d-%H%M%S")
    policy_name = policy_path.stem  # Get filename without extension
    log_filename = f"error_log_{policy_name}_{ts}.txt"
    log_path = ERROR_LOG_DIR / log_filename
    
    with open(log_path, "w", encoding="utf-8") as f:
        f.write(f"Error Log for Policy: {policy_path.name}\n")
        f.write(f"Timestamp: {time.strftime('%Y-%m-%d %H:%M:%S')}\n")
        f.write("=" * 80 + "\n\n")
        f.write(error_message)
        f.write("\n")
    
    return log_path


def execute_policy(policy_file: str, group):
    """
    Load and execute a generated policy file.
    
    Args:
        policy_file: Path to the policy file (e.g., "policies/20251016-135314_wave-to-me.py")
        group: MoveGroupCommander object for the robot arm
        
    Returns:
        Tuple of (success: bool, error_log_path: Path or None)
    """
    policy_path = Path(policy_file)
    
    if not policy_path.exists():
        error_msg = f"Policy file not found: {policy_path}"
        rospy.logerr(error_msg)
        log_path = create_error_log(policy_path, error_msg)
        return False, log_path
    
    rospy.loginfo(f"Loading policy: {policy_path}")
    
    # Read and execute the policy file
    # This defines the run(group) function in the local namespace
    policy_code = policy_path.read_text(encoding='utf-8')
    namespace = {}
    
    try:
        exec(policy_code, namespace)
    except Exception as e:
        error_msg = f"Failed to load policy: {e}\n\n{traceback.format_exc()}"
        rospy.logerr(f"Failed to load policy: {e}")
        log_path = create_error_log(policy_path, error_msg)
        rospy.logerr(f"Error log saved to: {log_path}")
        return False, log_path
    
    if 'run' not in namespace:
        error_msg = "Policy file must define a run(group) function"
        rospy.logerr(error_msg)
        log_path = create_error_log(policy_path, error_msg)
        rospy.logerr(f"Error log saved to: {log_path}")
        return False, log_path
    
    rospy.loginfo("Policy loaded successfully")
    
    # Execute the policy
    rospy.loginfo("Executing policy...")
    
    # Capture both stdout and stderr during execution
    error_capture = io.StringIO()
    
    try:
        # Get the run function from the namespace
        run_func = namespace['run']
        
        # Call it with the group object
        # Capture any output/errors
        with contextlib.redirect_stderr(error_capture):
            run_func(group)
        
        rospy.loginfo("Policy execution completed successfully!")
        return True, None
        
    except Exception as e:
        # Capture the full traceback
        tb_str = traceback.format_exc()
        captured_errors = error_capture.getvalue()
        
        # Combine all error information
        error_msg = f"Policy execution failed with exception:\n\n{tb_str}"
        if captured_errors:
            error_msg += f"\n\nCaptured stderr:\n{captured_errors}"
        
        rospy.logerr(f"Policy execution failed: {e}")
        log_path = create_error_log(policy_path, error_msg)
        rospy.logerr(f"Error log saved to: {log_path}")
        
        return False, log_path


if __name__ == '__main__':
    # Check arguments
    if len(sys.argv) < 2:
        print("Usage: python3 execute_policy.py <policy_file>")
        print("Example: python3 execute_policy.py policies/20251016-135314_wave-to-me.py")
        sys.exit(1)
    
    policy_file = sys.argv[1]
    
    # Initialize ROS node
    rospy.init_node('execute_policy', anonymous=True)
    rospy.loginfo("Initializing execute_policy node...")
    
    # Initialize MoveIt
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.loginfo("MoveIt initialized")
    
    # Create MoveGroupCommander for the right arm
    group = moveit_commander.MoveGroupCommander("right_arm")
    rospy.loginfo(f"MoveGroupCommander created for 'right_arm'")
    rospy.loginfo(f"Planning frame: {group.get_planning_frame()}")
    rospy.loginfo(f"End effector: {group.get_end_effector_link()}")
    
    # Execute the policy
    success, error_log_path = execute_policy(policy_file, group)
    
    # Clean up
    moveit_commander.roscpp_shutdown()
    
    if success:
        rospy.loginfo("Policy execution successful!")
        sys.exit(0)
    else:
        rospy.logerr("Policy execution failed!")
        if error_log_path:
            rospy.logerr(f"Error log available at: {error_log_path}")
        sys.exit(1)

