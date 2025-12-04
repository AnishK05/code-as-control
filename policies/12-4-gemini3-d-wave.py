from geometry_msgs.msg import Pose, Point, Quaternion
import rospy

def run(group):
    """
    Executes a waving motion 5 times quickly.
    
    Strategy:
    1. Move the arm to a central, elevated position suitable for waving.
    2. Oscillate the end-effector along the Y-axis (side-to-side) 5 times.
    3. 'Quickly' is achieved by not adding sleep delays between waypoints.
    """
    
    # Define a standard orientation for the wave
    # Using the identity quaternion as a safe default based on instructions
    orientation = Quaternion(0, 0, 0, 1)
    
    # 1. Move to starting position (Center, High)
    # Position: X=0.6 (forward), Y=0.0 (center), Z=0.45 (high/shoulder level)
    start_pose = Pose()
    start_pose.position = Point(0.6, 0.0, 0.45)
    start_pose.orientation = orientation
    
    group.set_pose_target(start_pose)
    group.go(wait=True)
    group.stop()
    group.clear_pose_targets()
    
    # 2. Perform the wave 5 times (Cycles of Left -> Right)
    # Amplitude: +/- 0.15m from center (Total width 30cm)
    # No sleeps included to satisfy "quickly" requirement
    
    for i in range(5):
        # Move Left (Positive Y)
        left_pose = Pose()
        left_pose.position = Point(0.6, 0.15, 0.45)
        left_pose.orientation = orientation
        
        group.set_pose_target(left_pose)
        group.go(wait=True)
        group.stop()
        group.clear_pose_targets()
        
        # Move Right (Negative Y)
        right_pose = Pose()
        right_pose.position = Point(0.6, -0.15, 0.45)
        right_pose.orientation = orientation
        
        group.set_pose_target(right_pose)
        group.go(wait=True)
        group.stop()
        group.clear_pose_targets()

    # 3. Return to center position to finish the gesture cleanly
    final_pose = Pose()
    final_pose.position = Point(0.6, 0.0, 0.45)
    final_pose.orientation = orientation
    
    group.set_pose_target(final_pose)
    group.go(wait=True)
    group.stop()
    group.clear_pose_targets()
