from typing import List
from typing import Tuple

import numpy as np
import rospy


def calculate_max_displacement(point1: np.ndarray, point2: np.ndarray) -> float:
    """
    Calculates the maximum absolute displacement between two joint configurations.

    Args:
        point1 (np.ndarray): The starting joint configuration.
        point2 (np.ndarray): The ending joint configuration.

    Returns:
        float: The maximum absolute displacement across all joints.
    """
    diff = point2 - point1
    return np.max(np.abs(diff))


def handle_no_movement(
    point1: np.ndarray, steps: int
) -> Tuple[List[rospy.Duration], np.ndarray, np.ndarray, np.ndarray]:
    """
    Handles the scenario where there is no movement between joint configurations.

    Args:
        point1 (np.ndarray): The starting joint configuration.
        steps (int): Number of interpolation steps.

    Returns:
        Tuple[List[rospy.Duration], np.ndarray, np.ndarray, np.ndarray]:
            - List of rospy.Duration objects (all zeros).
            - Joint positions replicated for each step.
            - Zero velocities for each joint at each step.
            - Zero accelerations for each joint at each step.
    """
    joint_positions = np.tile(point1, (steps, 1))
    times = [rospy.Duration(0) for _ in range(steps)]
    velocities = np.zeros((steps, len(point1)))
    accelerations = np.zeros((steps, len(point1)))
    return times, joint_positions, velocities, accelerations


def calculate_triangular_profile_times(D: float, joint_acc: float, steps: int) -> List[float]:
    """
    Calculates the timing for a triangular velocity profile.

    Args:
        D (float): Total displacement.
        joint_acc (float): Maximum joint acceleration.
        steps (int): Number of interpolation steps.

    Returns:
        List[float]: List of time stamps for each waypoint.
    """
    total_time = 2 * np.sqrt(D / joint_acc)
    distances = np.linspace(0, D, steps)
    times_float: List[float] = []

    for dist in distances:
        if dist <= D / 2:
            # Accelerate phase
            t = np.sqrt((2 * dist) / joint_acc)
        else:
            # Decelerate phase
            t = total_time - np.sqrt(2 * (D - dist) / joint_acc)
        times_float.append(t)

    return times_float


def calculate_trapezoidal_profile_times(D: float, joint_vel: float, joint_acc: float, steps: int) -> List[float]:
    """
    Calculates the timing for a trapezoidal velocity profile.

    Args:
        D (float): Total displacement.
        joint_vel (float): Maximum joint velocity.
        joint_acc (float): Maximum joint acceleration.
        steps (int): Number of interpolation steps.

    Returns:
        List[float]: List of time stamps for each waypoint.
    """
    D_acc = (joint_vel**2) / (2 * joint_acc)
    T_a = joint_vel / joint_acc
    D_a = D_acc  # Distance covered during acceleration or deceleration
    D_c = D - 2 * D_a  # Distance covered during cruising

    # Handle cases where cruising distance becomes negative
    if D_c < 0:
        D_c = 0.0
    T_c = D_c / joint_vel if D_c > 0 else 0.0
    total_time = 2 * T_a + T_c

    distances = np.linspace(0, D, steps)
    times_float: List[float] = []

    for dist in distances:
        if dist <= D_a:
            # Accelerate phase
            t = np.sqrt(2 * dist / joint_acc)
        elif dist <= (D_a + D_c):
            # Cruise phase
            t = T_a + (dist - D_a) / joint_vel
        else:
            # Decelerate phase
            dist_end = D - dist
            t = total_time - np.sqrt(2 * dist_end / joint_acc)
        times_float.append(t)

    return times_float


def interpolate_joint_positions(point1: np.ndarray, point2: np.ndarray, steps: int) -> np.ndarray:
    """
    Linearly interpolates joint positions between two configurations.

    Args:
        point1 (np.ndarray): The starting joint configuration.
        point2 (np.ndarray): The ending joint configuration.
        steps (int): Number of interpolation steps.

    Returns:
        np.ndarray: Array of interpolated joint positions with shape (steps, num_joints).
    """
    diff = point2 - point1
    fraction = np.linspace(0, 1, steps)
    joint_positions = point1 + np.outer(fraction, diff)
    return joint_positions


def calculate_finite_differences(
    joint_positions: np.ndarray, times_float: List[float]
) -> Tuple[np.ndarray, np.ndarray]:
    """
    Calculates velocities and accelerations using finite differences.

    Args:
        joint_positions (np.ndarray): Array of joint positions over time.
        times_float (List[float]): List of time stamps corresponding to each position.

    Returns:
        Tuple[np.ndarray, np.ndarray]:
            - Velocities calculated using numerical gradients.
            - Accelerations calculated using numerical gradients of velocities.
    """
    times_s = np.array(times_float)
    # Prevent division by zero in gradient calculation
    times_s = np.where(times_s == 0, 1e-6, times_s)
    joint_velocities = np.gradient(joint_positions, times_s, axis=0)
    joint_accelerations = np.gradient(joint_velocities, times_s, axis=0)
    return joint_velocities, joint_accelerations


def parametrize_joint_trajectory(
    point1: List[float],
    point2: List[float],
    steps: int,
    joint_vel: float,
    joint_acc: float,
) -> Tuple[
    List[rospy.Duration],
    np.ndarray,
    np.ndarray,
    np.ndarray,
]:
    """
    Parameterizes a joint trajectory between two configurations using a trapezoidal velocity profile.

    This function computes the timing, joint positions, velocities, and accelerations for a trajectory
    between two joint configurations based on specified velocity and acceleration constraints.

    Steps:
        1. Find the largest joint displacement D.
        2. Determine whether to use a triangular or trapezoidal velocity profile.
        3. Compute time stamps for each waypoint based on the selected profile.
        4. Linearly interpolate joint positions between the two configurations.
        5. Approximate velocities and accelerations using finite differences.

    Args:
        point1 (List[float]): The starting joint configuration.
        point2 (List[float]): The ending joint configuration.
        steps (int): Number of interpolation steps.
        joint_vel (float): Maximum joint velocity.
        joint_acc (float): Maximum joint acceleration.

    Returns:
        Tuple[
            List[rospy.Duration],
            np.ndarray,
            np.ndarray,
            np.ndarray,
        ]:
            - times: List of rospy.Duration objects for each waypoint.
            - joint_positions_over_time: Array of joint positions at each step.
            - joint_velocities_over_time: Array of joint velocities at each step (approximated).
            - joint_accelerations_over_time: Array of joint accelerations at each step (approximated).
    """
    point1_np = np.array(point1, dtype=np.float64)
    point2_np = np.array(point2, dtype=np.float64)
    D = calculate_max_displacement(point1_np, point2_np)

    if D < 1e-6:
        rospy.loginfo("No movement detected. Generating zeroed trajectory profiles.")
        return handle_no_movement(point1_np, steps)

    D_acc = (joint_vel**2) / (2 * joint_acc)

    if D < 2 * D_acc:
        rospy.loginfo("Using triangular velocity profile.")
        times_float = calculate_triangular_profile_times(D, joint_acc, steps)
    else:
        rospy.loginfo("Using trapezoidal velocity profile.")
        times_float = calculate_trapezoidal_profile_times(D, joint_vel, joint_acc, steps)

    times = [rospy.Duration(t) for t in times_float]
    joint_positions = interpolate_joint_positions(point1_np, point2_np, steps)
    joint_velocities, joint_accelerations = calculate_finite_differences(joint_positions, times_float)

    return times, joint_positions, joint_velocities, joint_accelerations
