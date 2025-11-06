from geometry_msgs.msg import Pose, Point, Quaternion
import rospy

def run(group):
    # Define the "high five" pose
    # Position: Forward, slightly to the side, and at a reasonable height for a high five
    # Orientation: Hand pointing upwards, palm facing forward.
    # A 90-degree rotation around the y-axis (pitch) orients the palm forward.
    # A 90-degree rotation around the x-axis (roll) orients the hand vertically.
    # Combining these can be complex, so a simpler approach is to orient the palm forward.
    # A 90-degree rotation around the y-axis is (x=0, y=0.707, z=0, w=0.707)
    high_five_pose = Pose()
    high_five_pose.position = Point(0.6, -0.2, 0.4)
    high_five_pose.orientation = Quaternion(0.0, 0.707, 0.0, 0.707)

    # Move to the high five position
    group.set_pose_target(high_five_pose)
    group.go(wait=True)
    group.stop()
    group.clear_pose_targets()

    # Hold the pose for a moment
    rospy.sleep(2.0)

    # Define a safe retraction pose
    # Move back slightly to a neutral, ready position
    retract_pose = Pose()
    retract_pose.position = Point(0.5, 0.0, 0.3)
    retract_pose.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)

    # Move to the retraction position
    group.set_pose_target(retract_pose)
    group.go(wait=True)
    group.stop()
    group.clear_pose_targets()

    rospy.sleep(1.0)
