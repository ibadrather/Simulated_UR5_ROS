from typing import List
from typing import Tuple

import numpy as np
import PyKDL as KDL
import rospy
from geometry_msgs.msg import Pose


def pose_to_kdl_frame(pose: Pose) -> KDL.Frame:
    """
    Converts a ROS Pose message to a PyKDL Frame.

    Args:
        pose (Pose): The ROS Pose message to convert.

    Returns:
        KDL.Frame: The corresponding PyKDL Frame.
    """
    rotation = KDL.Rotation.Quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
    vector = KDL.Vector(pose.position.x, pose.position.y, pose.position.z)
    return KDL.Frame(rotation, vector)


def interpolate_position(start: Pose, end: Pose, t: float) -> Tuple[float, float, float]:
    """
    Linearly interpolates between the positions of two poses.

    Args:
        start (Pose): The starting Pose.
        end (Pose): The ending Pose.
        t (float): Interpolation factor between 0 and 1.

    Returns:
        Tuple[float, float, float]: The interpolated (x, y, z) position.
    """
    x = (1 - t) * start.position.x + t * end.position.x
    y = (1 - t) * start.position.y + t * end.position.y
    z = (1 - t) * start.position.z + t * end.position.z
    return x, y, z


def interpolate_poses(start: Pose, end: Pose, steps: int) -> List[Pose]:
    """
    Generates a list of interpolated poses between a start and end pose.

    The orientation is kept constant for simplicity.

    Args:
        start (Pose): The starting Pose.
        end (Pose): The ending Pose.
        steps (int): Number of interpolation steps.

    Returns:
        List[Pose]: A list of interpolated Pose messages.
    """
    waypoints: List[Pose] = []
    for t in np.linspace(0, 1, steps):
        interp_pose = Pose()
        interp_pose.position.x, interp_pose.position.y, interp_pose.position.z = interpolate_position(start, end, t)
        interp_pose.orientation = start.orientation  # Keeping orientation constant
        waypoints.append(interp_pose)
    return waypoints


def distance_between_poses(p1: Pose, p2: Pose) -> float:
    """
    Calculates the Euclidean distance between two poses.

    Args:
        p1 (Pose): The first Pose.
        p2 (Pose): The second Pose.

    Returns:
        float: The Euclidean distance between p1 and p2.
    """
    dx = p2.position.x - p1.position.x
    dy = p2.position.y - p1.position.y
    dz = p2.position.z - p1.position.z
    return np.sqrt(dx * dx + dy * dy + dz * dz)


def calculate_triangle_profile(
    D: float, linear_vel: float, linear_acc: float, num_waypoints: int
) -> Tuple[List[rospy.Duration], List[float], List[float]]:
    """
    Generates timing, velocity, and acceleration profiles for a triangular trajectory.

    Args:
        D (float): Total distance of the trajectory.
        linear_vel (float): Maximum linear velocity.
        linear_acc (float): Maximum linear acceleration.
        num_waypoints (int): Number of waypoints in the trajectory.

    Returns:
        Tuple[List[rospy.Duration], List[float], List[float]]: Lists of times, velocities, and accelerations.
    """
    total_time = 2 * np.sqrt(D / linear_acc)
    distances = np.linspace(0, D, num_waypoints)
    times: List[rospy.Duration] = []

    for dist in distances:
        if dist <= D / 2:
            t = np.sqrt((2 * dist) / linear_acc)
        else:
            t = total_time - np.sqrt(2 * (D - dist) / linear_acc)
        times.append(rospy.Duration(t))

    times_float = [t.to_sec() for t in times]
    velocities = np.gradient(distances, times_float).tolist()
    accelerations = np.gradient(velocities, times_float).tolist()

    return times, velocities, accelerations


def calculate_trapezoidal_profile(
    D: float, linear_vel: float, linear_acc: float, num_waypoints: int
) -> Tuple[List[rospy.Duration], List[float], List[float]]:
    """
    Generates timing, velocity, and acceleration profiles for a trapezoidal trajectory.

    Args:
        D (float): Total distance of the trajectory.
        linear_vel (float): Maximum linear velocity.
        linear_acc (float): Maximum linear acceleration.
        num_waypoints (int): Number of waypoints in the trajectory.

    Returns:
        Tuple[List[rospy.Duration], List[float], List[float]]: Lists of times, velocities, and accelerations.
    """
    T_a = linear_vel / linear_acc
    D_a = (linear_vel**2) / (2 * linear_acc)
    D_c = D - 2 * D_a

    if D_c < 0:
        D_c = 0
    T_c = D_c / linear_vel
    total_time = 2 * T_a + T_c
    distances = np.linspace(0, D, num_waypoints)
    times: List[rospy.Duration] = []

    for dist in distances:
        if dist <= D_a:
            t = np.sqrt(2 * dist / linear_acc)
        elif dist <= (D_a + D_c):
            t = T_a + (dist - D_a) / linear_vel
        else:
            dist_end = D - dist
            t = total_time - np.sqrt(2 * dist_end / linear_acc)
        times.append(rospy.Duration(t))

    times_float = [t.to_sec() for t in times]
    velocities = np.gradient(distances, times_float).tolist()
    accelerations = np.gradient(velocities, times_float).tolist()

    return times, velocities, accelerations


def parametrize_trajectory(
    waypoints: List[Pose], linear_vel: float, linear_acc: float
) -> Tuple[List[rospy.Duration], List[float], List[float]]:
    """
    Parameterizes a trajectory based on waypoints, linear velocity, and acceleration.

    Determines whether to use a triangular or trapezoidal velocity profile based on the total distance.

    Args:
        waypoints (List[Pose]): The list of Pose waypoints defining the trajectory.
        linear_vel (float): The desired linear velocity.
        linear_acc (float): The desired linear acceleration.

    Returns:
        Tuple[List[rospy.Duration], List[float], List[float]]:
            - List of rospy.Duration objects representing the time at each waypoint.
            - List of velocities at each waypoint.
            - List of accelerations at each waypoint.
    """
    if not waypoints:
        rospy.logwarn("Empty waypoints list provided to parametrize_trajectory.")
        return [], [], []

    start_pose = waypoints[0]
    end_pose = waypoints[-1]
    D = distance_between_poses(start_pose, end_pose)

    if D < 1e-6:
        rospy.loginfo("Total distance is negligible. Returning zeroed profiles.")
        times = [rospy.Duration(0) for _ in waypoints]
        velocities = [0.0 for _ in waypoints]
        accelerations = [0.0 for _ in waypoints]
        return times, velocities, accelerations

    D_acc = (linear_vel**2) / (2 * linear_acc)

    if D < 2 * D_acc:
        rospy.loginfo("Using triangular velocity profile.")
        return calculate_triangle_profile(D, linear_vel, linear_acc, len(waypoints))
    else:
        rospy.loginfo("Using trapezoidal velocity profile.")
        return calculate_trapezoidal_profile(D, linear_vel, linear_acc, len(waypoints))
