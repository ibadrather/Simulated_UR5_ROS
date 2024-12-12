#!/usr/bin/env python3

import rospy
import kdl_parser_py.urdf
from PyKDL import ChainFkSolverPos_recursive, ChainIkSolverPos_LMA, JntArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
import numpy as np
import PyKDL as KDL
from typing import List
from tf.transformations import quaternion_slerp

# Define UR5 joint names
JOINT_NAMES = [
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
]


# Function to create a Pose message
def create_pose(x: float, y: float, z: float, w: float, wx: float, wy: float, wz: float) -> Pose:
    """
    Create a Pose message with the specified position and orientation.
    """
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z
    pose.orientation.w = w
    pose.orientation.x = wx
    pose.orientation.y = wy
    pose.orientation.z = wz
    return pose


# Function to convert Pose to KDL Frame
def pose_to_kdl_frame(pose: Pose) -> KDL.Frame:
    """
    Convert geometry_msgs/Pose to PyKDL Frame.
    """
    return KDL.Frame(
        KDL.Rotation.Quaternion(
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w
        ),
        KDL.Vector(pose.position.x, pose.position.y, pose.position.z)
    )


# Function to interpolate between two poses
def interpolate_poses(start: Pose, end: Pose, steps: int) -> List[Pose]:
    """
    Generate interpolated Cartesian poses between start and end with the specified number of steps.
    """
    waypoints = []
    for t in np.linspace(0, 1, steps):
        interp_pose = Pose()
        interp_pose.position.x = (1 - t) * start.position.x + t * end.position.x
        interp_pose.position.y = (1 - t) * start.position.y + t * end.position.y
        interp_pose.position.z = (1 - t) * start.position.z + t * end.position.z
        interp_pose.orientation = start.orientation  # Assumes constant orientation
        waypoints.append(interp_pose)
    return waypoints


# Function to initialize KDL solvers
def initialize_kdl_solvers(
    robot_description: str, base_link: str, end_effector_link: str
):
    """
    Initialize the KDL solvers for forward and inverse kinematics.
    """
    ok, tree = kdl_parser_py.urdf.treeFromString(robot_description)
    if not ok:
        rospy.logerr("Failed to parse URDF")
        return None, None, None, None
    kdl_chain = tree.getChain(base_link, end_effector_link)
    fk_solver = ChainFkSolverPos_recursive(kdl_chain)
    ik_solver = ChainIkSolverPos_LMA(kdl_chain)
    num_joints = kdl_chain.getNrOfJoints()
    return kdl_chain, fk_solver, ik_solver, num_joints


# Function to calculate joint trajectory
def calculate_joint_trajectory(
    ik_solver, kdl_chain, waypoints: List[Pose], initial_joints: JntArray
) -> List[List[float]]:
    """
    Calculate the joint trajectory for the given waypoints using the IK solver.
    """
    num_joints = kdl_chain.getNrOfJoints()
    joint_trajectory = []
    for waypoint in waypoints:
        frame = pose_to_kdl_frame(waypoint)
        waypoint_joints = JntArray(num_joints)
        if ik_solver.CartToJnt(initial_joints, frame, waypoint_joints) < 0:
            rospy.logerr("IK failed for a waypoint")
            return None
        joint_trajectory.append([waypoint_joints[i] for i in range(num_joints)])
    return joint_trajectory


# Function to publish joint trajectory
def publish_trajectory(
    joint_trajectory: List[List[float]], joint_names: List[str], publisher
):
    """
    Publish the calculated joint trajectory as a ROS message.
    """
    trajectory_msg = JointTrajectory()
    trajectory_msg.joint_names = joint_names

    for i, joints in enumerate(joint_trajectory):
        point = JointTrajectoryPoint()
        point.positions = joints
        point.velocities = [0.1] * len(joints)
        point.accelerations = [0.1] * len(joints)
        point.time_from_start = rospy.Duration(i * 0.1)
        trajectory_msg.points.append(point)

    publisher.publish(trajectory_msg)
    rospy.loginfo("Trajectory published")


# Function to handle forward kinematics logging
def log_fk_positions(fk_solver, joint_positions: JntArray, rate: rospy.Rate):
    """
    Log the end effector's position and orientation using forward kinematics.
    """
    while not rospy.is_shutdown():
        end_effector_frame = KDL.Frame()
        if fk_solver.JntToCart(joint_positions, end_effector_frame) >= 0:
            position = end_effector_frame.p
            rotation = end_effector_frame.M
            quaternion = rotation.GetQuaternion()
            # rospy.loginfo(
            #     f"End Effector Position: x={position.x() :.2f}, y={position.y() :.2f}, z={position.z() :.2f}\n"
            #     f"Orientation (quaternion): x={quaternion[0] :.2f}, y={quaternion[1] :.2f}, "
            #     f"z={quaternion[2] :.2f}, w={quaternion[3] :.2f}"
            # )
        else:
            rospy.logwarn("Failed to compute FK for the current joint positions")
        rate.sleep()


def main():
    import os
    os.system("clear")


    rospy.init_node("ur5_motion_node")

    # Load robot description and initialize KDL solvers
    robot_description = rospy.get_param("robot_description")
    base_link = "base_link"
    end_effector_link = "wrist_3_link"
    kdl_chain, fk_solver, ik_solver, num_joints = initialize_kdl_solvers(
        robot_description, base_link, end_effector_link
    )
    if not kdl_chain:
        return

    joint_positions = JntArray(num_joints)

    # Subscribe to joint states
    def joint_states_callback(msg: JointState):
        """Callback to update joint positions from the /joint_states topic."""
        if len(msg.position) >= num_joints:
            for i in range(num_joints):
                joint_positions[i] = msg.position[i]

    rospy.Subscriber("/joint_states", JointState, joint_states_callback)

    # Define start and end poses
    start_pose = create_pose(0.3, 0.6, 0.1, 1.0, 0.0, 0.0, 0.0)
    end_pose = create_pose(0.3, -0.6, 0.1, 1.0, 0.0, 0.0, 0.0)

    # Solve IK for waypoints
    waypoints = interpolate_poses(start_pose, end_pose, 200)

    initial_joints = JntArray(num_joints)
    joint_trajectory = calculate_joint_trajectory(
        ik_solver, kdl_chain, waypoints, initial_joints
    )
    if not joint_trajectory:
        return

    # Publish trajectory
    trajectory_pub = rospy.Publisher(
        "/eff_joint_traj_controller/command", JointTrajectory, queue_size=10
    )
    rospy.sleep(1)
    publish_trajectory(joint_trajectory, JOINT_NAMES, trajectory_pub)

    # Log forward kinematics
    rate = rospy.Rate(10)
    log_fk_positions(fk_solver, joint_positions, rate)


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
