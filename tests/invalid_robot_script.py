from geometry_msgs.msg import Pose, Point, Quaternion
import rospy

def run(group):
    # This uses an invalid API method
    pose = Pose()
    pose.position = Point(0.6, 0.0, 0.3)
    pose.orientation = Quaternion(0, 0, 0, 1)
    
    group.set_pose_target(pose)
    group.fly_to_moon()  # Invalid method!
    group.stop()

