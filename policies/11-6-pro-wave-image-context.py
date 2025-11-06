from geometry_msgs.msg import Pose, Point, Quaternion
import rospy

def run(group):
    """
    Raises the Sawyer robot arm and performs a slow wave three times.
    """
    # 1. Raise the arm to a waving position
    # This position is in front of the robot and raised up, like a hand ready to wave.
    pose_start_wave = Pose()
    pose_start_wave.position = Point(0.6, -0.2, 0.4)
    pose_start_wave.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)
    
    group.set_pose_target(pose_start_wave)
    group.go(wait=True)
    group.stop()
    group.clear_pose_targets()
    
    # A short pause after raising the arm
    rospy.sleep(1.0)

    # Define the two extreme points of the wave motion (left and right)
    # The wave will move side-to-side by changing the 'y' coordinate.
    
    # Wave to the right (from the robot's perspective)
    pose_wave_right = Pose()
    pose_wave_right.position = Point(0.6, -0.35, 0.4)
    pose_wave_right.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)

    # Wave to the left (from the robot's perspective)
    pose_wave_left = Pose()
    pose_wave_left.position = Point(0.6, -0.05, 0.4)
    pose_wave_left.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)

    # 2. Perform the wave 3 times
    # The slow speed is achieved by giving MoveIt enough time to plan and execute
    # smooth paths between the waypoints, combined with rospy.sleep pauses.
    for _ in range(3):
        # Move to the right
        group.set_pose_target(pose_wave_right)
        group.go(wait=True)
        group.stop()
        group.clear_pose_targets()
        rospy.sleep(0.5)

        # Move to the left
        group.set_pose_target(pose_wave_left)
        group.go(wait=True)
        group.stop()
        group.clear_pose_targets()
        rospy.sleep(0.5)

    # 3. Return to the initial raised position after waving
    group.set_pose_target(pose_start_wave)
    group.go(wait=True)
    group.stop()
    group.clear_pose_targets()
    rospy.sleep(1.0)
