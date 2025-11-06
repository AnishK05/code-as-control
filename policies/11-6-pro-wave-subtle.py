from geometry_msgs.msg import Pose, Point, Quaternion
import rospy

def run(group):
    """
    Performs a slow, subtle wave three times.
    """
    # 1. Move to a starting position for the wave (hand raised)
    start_pose = Pose()
    start_pose.position = Point(0.6, -0.3, 0.4)
    start_pose.orientation = Quaternion(0, 0, 0, 1)
    
    group.set_pose_target(start_pose)
    group.go(wait=True)
    group.stop()
    group.clear_pose_targets()
    
    # Pause before starting the wave
    rospy.sleep(1.0)

    # Define the two points for the subtle side-to-side wave motion
    wave_pose_1 = Pose()
    wave_pose_1.position = Point(0.6, -0.25, 0.4) # Wave to one side
    wave_pose_1.orientation = Quaternion(0, 0, 0, 1)

    wave_pose_2 = Pose()
    wave_pose_2.position = Point(0.6, -0.35, 0.4) # Wave to the other side
    wave_pose_2.orientation = Quaternion(0, 0, 0, 1)

    # 2. Execute the wave motion 3 times
    for _ in range(3):
        # Move to the first wave point
        group.set_pose_target(wave_pose_1)
        group.go(wait=True)
        group.stop()
        group.clear_pose_targets()
        rospy.sleep(0.5) # Slow pause for a slow wave

        # Move to the second wave point
        group.set_pose_target(wave_pose_2)
        group.go(wait=True)
        group.stop()
        group.clear_pose_targets()
        rospy.sleep(0.5) # Slow pause

    # 3. Return to the initial wave start position to finish
    group.set_pose_target(start_pose)
    group.go(wait=True)
    group.stop()
    group.clear_pose_targets()
