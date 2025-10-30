from geometry_msgs.msg import Pose, Point, Quaternion
import rospy

def run(group):
    # Move to a neutral waving start position
    start_pose = Pose()
    start_pose.position = Point(0.7, 0.0, 0.3)
    start_pose.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)
    group.set_pose_target(start_pose)
    group.go(wait=True)
    group.stop()
    group.clear_pose_targets()
    rospy.sleep(0.5)

    # Define the two points for the wave
    wave_left_pose = Pose()
    wave_left_pose.position = Point(0.7, -0.15, 0.3)
    wave_left_pose.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)

    wave_right_pose = Pose()
    wave_right_pose.position = Point(0.7, 0.15, 0.3)
    wave_right_pose.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)

    # Perform the wave motion three times
    for _ in range(3):
        # Move left
        group.set_pose_target(wave_left_pose)
        group.go(wait=True)
        group.stop()
        group.clear_pose_targets()
        rospy.sleep(0.5)

        # Move right
        group.set_pose_target(wave_right_pose)
        group.go(wait=True)
        group.stop()
        group.clear_pose_targets()
        rospy.sleep(0.5)

    # Return to the initial start position
    group.set_pose_target(start_pose)
    group.go(wait=True)
    group.stop()
    group.clear_pose_targets()
