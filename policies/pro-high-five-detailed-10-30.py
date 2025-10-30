from geometry_msgs.msg import Pose, Point, Quaternion
import rospy

def run(group):
    # Define the high-five pose
    # Position: In front of the robot at a reasonable height for a high-five
    # Orientation: Palm facing forward (vertical)
    pose = Pose()
    pose.position = Point(0.65, 0.0, 0.3)
    pose.orientation = Quaternion(0.0, 0.707, 0.0, 0.707)

    # Set the target pose
    group.set_pose_target(pose)

    # Plan and execute the motion
    group.go(wait=True)

    # Stop any residual motion
    group.stop()

    # Clear the pose target
    group.clear_pose_targets()

    # Hold the position for 5 seconds as requested
    rospy.sleep(5.0)
