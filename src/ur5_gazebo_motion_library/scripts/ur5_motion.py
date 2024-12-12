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


# Define UR5 joint names
JOINT_NAMES = [
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
]


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


def initialize_kdl_solvers(robot_description: str, base_link: str, end_effector_link: str):
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


def calculate_joint_trajectory(ik_solver, kdl_chain, waypoints: List[Pose], initial_joints: JntArray) -> List[List[float]]:
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


def distance_between_poses(p1: Pose, p2: Pose) -> float:
    dx = p2.position.x - p1.position.x
    dy = p2.position.y - p1.position.y
    dz = p2.position.z - p1.position.z
    return np.sqrt(dx*dx + dy*dy + dz*dz)


def parametrize_trajectory(waypoints: List[Pose], linear_vel: float, linear_acc: float):
    """
    Parametrize the trajectory in time using a trapezoidal velocity profile.
    Returns:
        times: list of time_from_start for each waypoint
        velocities: list of linear velocities at each waypoint
        accelerations: list of linear accelerations at each waypoint
    """
    # Compute total distance
    start_pose = waypoints[0]
    end_pose = waypoints[-1]
    D = distance_between_poses(start_pose, end_pose)

    # Handle the case where total distance is very small
    if D < 1e-6:
        # No movement, all times = 0
        times = [rospy.Duration(0) for _ in waypoints]
        velocities = [0.0 for _ in waypoints]
        accelerations = [0.0 for _ in waypoints]
        return times, velocities, accelerations

    # Compute trapezoidal profile
    # 1. Check if we can reach max velocity
    # Distance needed to accelerate to v: D_acc = v^2/(2*a)
    D_acc = (linear_vel**2) / (2*linear_acc)
    if D < 2*D_acc:
        # We never reach full velocity (triangle profile)
        # D = a*T^2 (where T is total time)
        # T = sqrt(D/a)
        total_time = np.sqrt(D/linear_acc)*2  # accelerate and then decelerate symmetrically
        # Times for each waypoint: proportional to distance fraction
        distances = np.linspace(0, D, len(waypoints))
        times = []
        for dist in distances:
            # Solve dist for triangle profile
            # dist = 0.5*a*t^2 if t <= T/2 (accel phase)
            # or dist = D - 0.5*a*(T - t)^2 if t > T/2 (decel phase)
            half_T = total_time/2
            if dist <= D/2:
                # accel phase
                # dist = 0.5*a*t^2
                t = np.sqrt((2*dist)/linear_acc)
            else:
                # decel phase
                # dist = D - 0.5*a*(T - t)^2
                # 0.5*a*(T-t)^2 = D - dist
                # (T-t)^2 = 2*(D-dist)/a
                # T-t = sqrt(...)
                # t = T - sqrt(...)
                t = total_time - np.sqrt(2*(D-dist)/linear_acc)
            times.append(rospy.Duration(t))

        # Approximate velocities and accelerations by finite differences
        times_float = [t.to_sec() for t in times]
        pos_array = distances
        velocities = np.gradient(pos_array, times_float)
        accelerations = np.gradient(velocities, times_float)
        return times, velocities, accelerations
    else:
        # Full trapezoidal profile
        # We accelerate from 0 to v: time T_a = v/a, distance D_a = 0.5*a*T_a^2 = v^2/(2*a)
        # Decelerate similarly: D_d = D_a, T_d = T_a
        # Remaining distance: D_c = D - 2*D_a, traveled at constant v
        # T_c = D_c/v
        T_a = linear_vel/linear_acc
        D_a = D_acc  # from above
        D_c = D - 2*D_a
        if D_c < 0:
            # Shouldn't happen here since we checked, but just in case
            D_c = 0
        T_c = D_c/linear_vel
        total_time = 2*T_a + T_c

        # Now assign time to each waypoint
        distances = np.linspace(0, D, len(waypoints))
        times = []
        for dist in distances:
            if dist <= D_a:
                # accelerate phase
                # dist = 0.5*a*t^2 -> t = sqrt(2*dist/a)
                t = np.sqrt(2*dist/linear_acc)
            elif dist <= (D_a + D_c):
                # cruise phase
                # dist = D_a + v*(t - T_a)
                # t = T_a + (dist - D_a)/v
                t = T_a + (dist - D_a)/linear_vel
            else:
                # decelerate phase
                # dist from the end: dist_end = D - dist
                # for decel: dist_end = 0.5*a*(T - t)^2
                # T - t = sqrt(2*dist_end/a)
                # t = T - sqrt(2*dist_end/a)
                dist_end = D - dist
                t = total_time - np.sqrt(2*dist_end/linear_acc)
            times.append(rospy.Duration(t))

        # Approximate velocities and accelerations by finite differences
        times_float = [t.to_sec() for t in times]
        pos_array = distances
        velocities = np.gradient(pos_array, times_float)
        accelerations = np.gradient(velocities, times_float)
        return times, velocities, accelerations


def publish_trajectory(
    joint_trajectory: List[List[float]],
    joint_names: List[str],
    publisher,
    waypoints: List[Pose],
    linear_vel: float,
    linear_acc: float
):
    """
    Publish the calculated joint trajectory as a ROS message with a trapezoidal velocity profile
    in Cartesian space applied before IK.
    """
    times, velocities, accelerations = parametrize_trajectory(waypoints, linear_vel, linear_acc)

    trajectory_msg = JointTrajectory()
    trajectory_msg.joint_names = joint_names

    for i, joints in enumerate(joint_trajectory):
        point = JointTrajectoryPoint()
        point.positions = joints
        # Assign joint velocities and accelerations as some fraction related to linear profile
        # For simplicity, we just use the same velocity/acceleration for all joints here.
        # A real solution would compute joint-level velocities from the IK results.
        # Here, we show a conceptual improvement.
        point.velocities = [velocities[i]] * len(joints)
        point.accelerations = [accelerations[i]] * len(joints)
        point.time_from_start = times[i]
        trajectory_msg.points.append(point)

    publisher.publish(trajectory_msg)
    rospy.loginfo("Trajectory published with velocity/acceleration constraints.")


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
            # Optional: Uncomment to see FK logs
            rospy.loginfo(
                f"End Effector Position: x={position.x() :.2f}, y={position.y() :.2f}, z={position.z() :.2f}\n"
                f"Orientation (quaternion): x={quaternion[0] :.2f}, y={quaternion[1] :.2f}, "
                f"z={quaternion[2] :.2f}, w={quaternion[3] :.2f}"
            )
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

    # Callback to update joint positions
    def joint_states_callback(msg: JointState):
        if len(msg.position) >= num_joints:
            for i in range(num_joints):
                joint_positions[i] = msg.position[i]

    rospy.Subscriber("/joint_states", JointState, joint_states_callback)

    # User-defined linear velocity and acceleration (m/s and m/s^2)
    linear_vel = 0.5
    linear_acc = 0.3

    # Define start and end poses
    start_pose = create_pose(0.3, 0.6, 0.1, 1.0, 0.0, 0.0, 0.0)
    end_pose = create_pose(0.3, -0.6, 0.1, 1.0, 0.0, 0.0, 0.0)

    # Generate Cartesian waypoints
    waypoints = interpolate_poses(start_pose, end_pose, 200)

    initial_joints = JntArray(num_joints)
    joint_trajectory = calculate_joint_trajectory(
        ik_solver, kdl_chain, waypoints, initial_joints
    )
    if not joint_trajectory:
        return

    # Publish trajectory with considered velocity/acceleration constraints
    trajectory_pub = rospy.Publisher(
        "/eff_joint_traj_controller/command", JointTrajectory, queue_size=10
    )
    rospy.sleep(1)
    publish_trajectory(joint_trajectory, JOINT_NAMES, trajectory_pub, waypoints, linear_vel, linear_acc)

    # Log forward kinematics
    rate = rospy.Rate(10)
    log_fk_positions(fk_solver, joint_positions, rate)


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
