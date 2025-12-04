from geometry_msgs.msg import Pose, Point, Quaternion
import rospy

def run(group):
    # Coordinate frame details:
    # Base frame: X forward, Y left, Z up
    # Drawing plane: X = 0.65 (vertical plane in front of robot)
    # Center of Figure-8: (0.65, 0.0, 0.3)
    # Dimensions: ~0.12m width, ~0.2m height
    
    # Standard orientation for Sawyer (gripper pointing down/forward)
    # Using Quaternion(0, 1, 0, 0) rotates the gripper 180 deg around X
    # This points the gripper down (-Z), which is a safe, versatile drawing orientation
    orient = Quaternion(0.0, 1.0, 0.0, 0.0)

    # Define waypoints for a figure-eight pattern (Vertical 8)
    # Sequence: Center -> Top Loop (CCW) -> Center -> Bottom Loop (CCW) -> Center
    # Smooth flow: Top-Left comes into Center, continues to Bottom-Left
    
    # Waypoints defined as Point(x, y, z)
    # Y is positive left, negative right
    # Z is positive up
    waypoints = [
        Point(0.65,  0.00, 0.30),  # Center
        Point(0.65, -0.06, 0.35),  # Top Right
        Point(0.65,  0.00, 0.40),  # Top Peak
        Point(0.65,  0.06, 0.35),  # Top Left
        Point(0.65,  0.00, 0.30),  # Center (Crossover)
        Point(0.65,  0.06, 0.25),  # Bottom Left
        Point(0.65,  0.00, 0.20),  # Bottom Peak
        Point(0.65, -0.06, 0.25),  # Bottom Right
        Point(0.65,  0.00, 0.30)   # Center (Complete)
    ]

    # Repeat the pattern 2 times as requested
    repetitions = 2

    # Move to start position first
    start_pose = Pose()
    start_pose.position = waypoints[0]
    start_pose.orientation = orient
    
    group.set_pose_target(start_pose)
    group.go(wait=True)
    group.stop()
    group.clear_pose_targets()
    
    # Execute the pattern
    for i in range(repetitions):
        for pt in waypoints:
            pose = Pose()
            pose.position = pt
            pose.orientation = orient
            
            group.set_pose_target(pose)
            group.go(wait=True)
            group.stop()
            group.clear_pose_targets()
            
            # Short sleep to pace the movement, though go(wait=True) handles most timing
            # Removing explicit sleep for smoother continuity between points
            
    # Final rest
    rospy.sleep(0.5)
