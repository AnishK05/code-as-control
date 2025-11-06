from geometry_msgs.msg import Pose, Point, Quaternion
import rospy

def run(group):
    # Define a pose that is at a safe, central location
    # with the end-effector oriented to point upwards.
    # A 90-degree rotation around the Y-axis points the tool's X-axis upwards.
    # Quaternion for 90-degree rotation around Y-axis: (x=0, y=sin(45), z=0, w=cos(45))
    # sin(45) = 0.707, cos(45) = 0.707
    
    pose = Pose()
    pose.position = Point(0.6, 0.0, 0.3)
    pose.orientation = Quaternion(0.0, 0.707, 0.0, 0.707)
    
    # Set the target, plan, and execute
    group.set_pose_target(pose)
    group.go(wait=True)
    
    # Ensure the robot has stopped and targets are cleared
    group.stop()
    group.clear_pose_targets()
    
    # Pause briefly after the movement
    rospy.sleep(1.0)
