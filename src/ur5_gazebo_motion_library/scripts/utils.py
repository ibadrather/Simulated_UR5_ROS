from geometry_msgs.msg import Pose

JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]


def create_pose(x: float, y: float, z: float, w: float, wx: float, wy: float, wz: float) -> Pose:
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z
    pose.orientation.w = w
    pose.orientation.x = wx
    pose.orientation.y = wy
    pose.orientation.z = wz
    return pose
