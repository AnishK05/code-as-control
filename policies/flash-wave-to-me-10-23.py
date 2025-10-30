from geometry_msgs.msg import Pose, Point, Quaternion
import rospy

def run(group):
    # Initial position (neutral)
    pose = Pose()
    pose.position = Point(0.6, 0.0, 0.3)
    pose.orientation = Quaternion(0, 0, 0, 1)
    group.set_pose_target(pose)
    group.go(wait=True)
    group.stop()
    group.clear_pose_targets()
    rospy.sleep(1.0)

    # Wave up
    pose.position = Point(0.6, 0.2, 0.4)
    pose.orientation = Quaternion(0, 0, 0, 1)
    group.set_pose_target(pose)
    group.go(wait=True)
    group.stop()
    group.clear_pose_targets()
    rospy.sleep(0.5)

    # Wave down
    pose.position = Point(0.6, 0.2, 0.35)
    pose.orientation = Quaternion(0, 0, 0, 1)
    group.set_pose_target(pose)
    group.go(wait=True)
    group.stop()
    group.clear_pose_targets()
    rospy.sleep(0.5)

    # Wave up again
    pose.position = Point(0.6, 0.2, 0.4)
    pose.orientation = Quaternion(0, 0, 0, 1)
    group.set_pose_target(pose)
    group.go(wait=True)
    group.stop()
    group.clear_pose_targets()
    rospy.sleep(0.5)

    # Return to neutral
    pose.position = Point(0.6, 0.0, 0.3)
    pose.orientation = Quaternion(0, 0, 0, 1)
    group.set_pose_target(pose)
    group.go(wait=True)
    group.stop()
    group.clear_pose_targets()
    rospy.sleep(1.0)