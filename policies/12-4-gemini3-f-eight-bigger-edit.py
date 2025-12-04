from geometry_msgs.msg import Pose, Point, Quaternion
import rospy

def run(group):
    # Coordinate frame details:
    # Base frame: X forward, Y left, Z up
    # Drawing plane: X = 0.65 (vertical plane in front of robot)
    # Center of Figure-8: (0.65, 0.0, 0.3)
    
    # Previous dimensions: ~0.12m width, ~0.2m height
    # New dimensions (Much Bigger): ~0.4m width, ~0.3m height
    # Y Range: [-0.20, 0.20] (was [-0.06, 0.06])
    # Z Range: [0.15, 0.45] (was [0.20, 0.40])
    
    # Standard orientation for Sawyer (gripper pointing down/forward)
    # Quaternion(0, 1, 0, 0) rotates the gripper 180 deg around X (pointing down -Z)
    orient = Quaternion(0.0, 1.0, 0.0, 0.0)

    # Define waypoints for a larger figure-eight pattern (Vertical 8)
    # Sequence: Center -> Top Loop (CCW) -> Center -> Bottom Loop (CCW) -> Center
    # Y is positive left, negative right
    # Z is positive up
    
    # Using larger offsets to satisfy "much bigger" request
    # Y offset: +/- 0.20m
    # Z offset: +/- 0.15m from center (0.3)
    
    waypoints = [
        Point(0.65,  0.00, 0.30),   # Center
        Point(0.65, -0.20, 0.375),  # Top Right (Wide)
        Point(0.65,  0.00, 0.45),   # Top Peak (High)
        Point(0.65,  0.20, 0.375),  # Top Left (Wide)
        Point(0.65,  0.00, 0.30),   # Center (Crossover)
        Point(0.65,  0.20, 0.225),  # Bottom Left (Wide)
        Point(0.65,  0.00, 0.15),   # Bottom Peak (Low)
        Point(0.65, -0.20, 0.225),  # Bottom Right (Wide)
        Point(0.65,  0.00, 0.30)    # Center (Complete)
    ]

    # Repeat the pattern 2 times
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
            
    # Final rest
    rospy.sleep(0.5)
