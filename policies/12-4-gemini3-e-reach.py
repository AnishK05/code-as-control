from geometry_msgs.msg import Pose, Point, Quaternion
import rospy

def run(group):
    # Set conservative speed and acceleration for safety as requested
    group.set_max_velocity_scaling_factor(0.3)
    group.set_max_acceleration_scaling_factor(0.3)

    # Attempt 1: Reach as far forward as possible within typical workspace bounds
    # x=0.9 is the upper bound of the typical workspace
    # y=0.0 centers the arm
    # z=0.2 maintains a safe height above potential obstacles/table
    target_pose = Pose()
    target_pose.position = Point(0.9, 0.0, 0.2)
    # Orientation: Quaternion(0, 1, 0, 0) typically points the gripper down (standard safe pose for Sawyer)
    target_pose.orientation = Quaternion(0.0, 1.0, 0.0, 0.0)

    group.set_pose_target(target_pose)
    success = group.go(wait=True)
    group.stop()
    group.clear_pose_targets()

    # Attempt 2: If the full extension was not possible/safe, move to a closer position
    if not success:
        # Retry with a reduced X value (closer to base)
        target_pose.position = Point(0.75, 0.0, 0.2)
        
        group.set_pose_target(target_pose)
        group.go(wait=True)
        group.stop()
        group.clear_pose_targets()

    rospy.sleep(1.0)
