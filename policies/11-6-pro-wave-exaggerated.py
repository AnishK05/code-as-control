from geometry_msgs.msg import Pose, Point, Quaternion
import rospy

def run(group):
    """
    Moves the Sawyer arm to a raised position and performs an exaggerated, slow wave three times.
    """
    # 1. Move to the starting position for the wave
    # This is a raised, central position in front of the robot.
    start_pose = Pose()
    start_pose.position = Point(0.6, 0.0, 0.4)
    start_pose.orientation = Quaternion(0, 0, 0, 1)
    
    group.set_pose_target(start_pose)
    group.go(wait=True)
    group.stop()
    group.clear_pose_targets()
    
    # A short pause before the wave starts
    rospy.sleep(1.0)

    # Define the two extreme points of the exaggerated wave
    # Left position (from robot's perspective)
    left_pose = Pose()
    left_pose.position = Point(0.6, -0.3, 0.4)
    left_pose.orientation = Quaternion(0, 0, 0, 1)

    # Right position (from robot's perspective)
    right_pose = Pose()
    right_pose.position = Point(0.6, 0.3, 0.4)
    right_pose.orientation = Quaternion(0, 0, 0, 1)

    # 2. Perform the wave motion 3 times
    for _ in range(3):
        # Move to the left
        group.set_pose_target(left_pose)
        group.go(wait=True)
        group.stop()
        group.clear_pose_targets()
        rospy.sleep(0.5) # Pause at the peak of the wave

        # Move to the right
        group.set_pose_target(right_pose)
        group.go(wait=True)
        group.stop()
        group.clear_pose_targets()
        rospy.sleep(0.5) # Pause at the other peak

    # 3. Return to the initial central position after waving
    group.set_pose_target(start_pose)
    group.go(wait=True)
    group.stop()
    group.clear_pose_targets()
    rospy.sleep(1.0)
