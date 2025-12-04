from geometry_msgs.msg import Pose, Point, Quaternion
import rospy

def run(group):
    """
    Executes a vertical circle drawing motion in front of the robot.
    Based on clarification: Vertical orientation (whiteboard style), 
    Large size (approx 0.15m radius), Center chest height, Medium pace.
    """
    
    # Fixed X depth for drawing on a vertical plane (forward reach)
    # Center of circle is roughly (0.65, 0.0, 0.35)
    x_depth = 0.65
    
    # Standard orientation (no rotation)
    draw_orient = Quaternion(0, 0, 0, 1)
    
    # 1. Move to start position (Top of the circle)
    # Center Z (0.35) + Radius (0.15) = 0.50
    start_point = Point(x_depth, 0.0, 0.50)
    
    pose_start = Pose()
    pose_start.position = start_point
    pose_start.orientation = draw_orient
    
    group.set_pose_target(pose_start)
    group.go(wait=True)
    group.stop()
    group.clear_pose_targets()
    
    # Short pause before starting the shape
    rospy.sleep(0.5)
    
    # 2. Define waypoints for the circle (Octagon approximation)
    # This creates a "smooth" enough circle using discrete moves
    # Going Counter-Clockwise: Top -> Left -> Bottom -> Right -> Top
    # Radius = 0.15m
    # 0.707 * 0.15 approx 0.11 for diagonals
    
    waypoints = [
        Point(x_depth, 0.11, 0.46),   # Top-Left
        Point(x_depth, 0.15, 0.35),   # Left
        Point(x_depth, 0.11, 0.24),   # Bottom-Left
        Point(x_depth, 0.00, 0.20),   # Bottom
        Point(x_depth, -0.11, 0.24),  # Bottom-Right
        Point(x_depth, -0.15, 0.35),  # Right
        Point(x_depth, -0.11, 0.46),  # Top-Right
        Point(x_depth, 0.00, 0.50)    # Back to Top
    ]
    
    # 3. Execute the drawing motion
    for pt in waypoints:
        target = Pose()
        target.position = pt
        target.orientation = draw_orient
        
        group.set_pose_target(target)
        group.go(wait=True)
        group.stop()
        group.clear_pose_targets()
        
    # Final pause to indicate completion
    rospy.sleep(0.5)
