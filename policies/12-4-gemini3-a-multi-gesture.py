from geometry_msgs.msg import Pose, Point, Quaternion
import rospy

def run(group):
    # Define a standard orientation for interaction (pointing forward/down)
    # Using (0, 1, 0, 0) (180 deg rotation about Y) is a common safe orientation for Sawyer
    # This keeps the gripper generally pointing downwards or forwards depending on configuration
    standard_orientation = Quaternion(0.0, 1.0, 0.0, 0.0)

    # 1. Wave to the user
    # Move to a central waving position
    wave_center = Pose()
    wave_center.position = Point(0.6, 0.0, 0.4)
    wave_center.orientation = standard_orientation
    
    group.set_pose_target(wave_center)
    group.go(wait=True)
    group.stop()
    group.clear_pose_targets()
    
    # Perform wave motion (side-to-side oscillation)
    # 3 cycles of waving
    for _ in range(3):
        # Move Right (Positive Y)
        wave_right = Pose()
        wave_right.position = Point(0.6, 0.15, 0.4)
        wave_right.orientation = standard_orientation
        group.set_pose_target(wave_right)
        group.go(wait=True)
        
        # Move Left (Negative Y)
        wave_left = Pose()
        wave_left.position = Point(0.6, -0.15, 0.4)
        wave_left.orientation = standard_orientation
        group.set_pose_target(wave_left)
        group.go(wait=True)

    # Return to center after waving
    group.set_pose_target(wave_center)
    group.go(wait=True)
    group.stop()
    group.clear_pose_targets()
    
    rospy.sleep(0.5)

    # 2. Give a High Five
    # Move to a higher, more forward position to simulate a high five
    # Coordinates: High Z (0.5), Forward X (0.8)
    high_five_pose = Pose()
    high_five_pose.position = Point(0.8, 0.0, 0.5)
    high_five_pose.orientation = standard_orientation
    
    group.set_pose_target(high_five_pose)
    group.go(wait=True)
    group.stop()
    group.clear_pose_targets()
    
    # Pause to allow the "High Five" interaction
    rospy.sleep(2.0)

    # 3. Return Home
    # Move to a compact, safe home position near the base
    home_pose = Pose()
    home_pose.position = Point(0.4, 0.0, 0.2)
    home_pose.orientation = standard_orientation
    
    group.set_pose_target(home_pose)
    group.go(wait=True)
    group.stop()
    group.clear_pose_targets()
