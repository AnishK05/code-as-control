from geometry_msgs.msg import Pose, Point, Quaternion
import rospy

def run(group):
    # Initial "home" position
    pose = Pose()
    pose.position = Point(0.6, 0.0, 0.3)
    pose.orientation = Quaternion(0, 0, 0, 1)
    group.set_pose_target(pose)
    group.go(wait=True)
    group.stop()
    group.clear_pose_targets()
    rospy.sleep(1.0)

    # Wave position 1 (arm up)
    pose.position = Point(0.6, -0.3, 0.4)
    pose.orientation = Quaternion(0, 0, 0, 1)
    group.set_pose_target(pose)
    group.go(wait=True)
    group.stop()
    group.clear_pose_targets()
    rospy.sleep(1.0)

    # Wave position 2 (arm slightly to the side)
    pose.position = Point(0.6, -0.2, 0.4)
    pose.orientation = Quaternion(0, 0, 0, 1)
    group.set_pose_target(pose)
    group.go(wait=True)
    group.stop()
    group.clear_pose_targets()
    rospy.sleep(0.5)

    # Wave position 3 (arm slightly to the other side)
    pose.position = Point(0.6, -0.4, 0.4)
    pose.orientation = Quaternion(0, 0, 0, 1)
    group.set_pose_target(pose)
    group.go(wait=True)
    group.stop()
    group.clear_pose_targets()
    rospy.sleep(0.5)

    # Wave position 2 again
    pose.position = Point(0.6, -0.2, 0.4)
    pose.orientation = Quaternion(0, 0, 0, 1)
    group.set_pose_target(pose)
    group.go(wait=True)
    group.stop()
