from geometry_msgs.msg import Pose, Point, Quaternion
import rospy

def run(group):
    # The error "unable to sample valid states for goal tree" indicates that the
    # planner could not find a valid inverse kinematics solution for the requested pose.
    # This is often due to an orientation that is difficult or impossible for the
    # robot's wrist to achieve at that specific position.
    # The original orientation Quaternion(0.0, 0.707, 0.0, 0.707) represents a pure
    # 90-degree pitch, which can be challenging.
    #
    # FIX: We will use a more achievable orientation that still presents the gripper
    # vertically for a high five. A combination of roll and pitch is often more
    # stable for the Sawyer arm. A 180-degree roll (around X) and a 90-degree
    # pitch (around Y) results in Quaternion(0.707, 0.707, 0.0, 0.0).
    # We also adjust the position slightly to be more central and reachable.
    
    # Define a more reachable "high five" pose
    high_five_pose = Pose()
    high_five_pose.position = Point(0.65, -0.15, 0.35)
    high_five_pose.orientation = Quaternion(0.707, 0.707, 0.0, 0.0)

    # Move to the high five position
    group.set_pose_target(high_five_pose)
    group.go(wait=True)
    group.stop()
    group.clear_pose_targets()

    # Hold the pose for a moment for the high five
    rospy.sleep(2.0)

    # Define a safe retraction pose
    # Move back to a neutral, ready position
    retract_pose = Pose()
    retract_pose.position = Point(0.5, 0.0, 0.3)
    # Use a neutral, downward-facing orientation
    retract_pose.orientation = Quaternion(0.0, 0.924, 0.0, 0.383)

    # Move to the retraction position
    group.set_pose_target(retract_pose)
    group.go(wait=True)
    group.stop()
    group.clear_pose_targets()

    rospy.sleep(1.0)
