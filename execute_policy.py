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


def execute_policy(policy_file: str):
    """
    Load and execute a generated policy file.
    
    Args:
        policy_file: Path to the policy file (e.g., "policies/20251016-135314_wave-to-me.py")
    """
    policy_path = Path(policy_file)
    
    if not policy_path.exists():
        rospy.logerr(f"Policy file not found: {policy_path}")
        return False
    
    rospy.loginfo(f"Loading policy: {policy_path}")
    
    # Read and execute the policy file
    # This defines the run(group) function in the local namespace
    policy_code = policy_path.read_text(encoding='utf-8')
    namespace = {}
    
    try:
        exec(policy_code, namespace)
    except Exception as e:
        rospy.logerr(f"Failed to load policy: {e}")
        return False
    
    if 'run' not in namespace:
        rospy.logerr("Policy file must define a run(group) function")
        return False
    
    rospy.loginfo("Policy loaded successfully")
    
    # Execute the policy
    rospy.loginfo("Executing policy...")
    try:
        # Get the run function from the namespace
        run_func = namespace['run']
        
        # Call it with the group object
        run_func(group)
        
        rospy.loginfo("Policy execution completed successfully!")
        return True
        
    except Exception as e:
        rospy.logerr(f"Policy execution failed: {e}")
        import traceback
        traceback.print_exc()
        return False


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
    success = execute_policy(policy_file)
    
    # Clean up
    moveit_commander.roscpp_shutdown()
    
    if success:
        rospy.loginfo("Policy execution successful!")
        sys.exit(0)
    else:
        rospy.logerr("Policy execution failed!")
        sys.exit(1)

