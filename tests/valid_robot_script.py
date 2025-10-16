from geometry_msgs.msg import Pose, Point, Quaternion
import rospy

def run(group):
    # Move arm to position
    pose = Pose()
    pose.position = Point(0.6, 0.0, 0.3)
    pose.orientation = Quaternion(0, 0, 0, 1)
    
    group.set_pose_target(pose)
    group.go(wait=True)
    group.stop()
    group.clear_pose_targets()
    
    rospy.sleep(1.0)

