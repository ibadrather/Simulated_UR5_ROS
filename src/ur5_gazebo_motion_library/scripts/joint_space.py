import numpy as np
from typing import List
import rospy


def parametrize_joint_trajectory(
    point1: List[float], point2: List[float], steps: int, joint_vel: float, joint_acc: float
):
    """
    Given two joint configurations and desired joint velocity/acceleration,
    create a time parameterization for a trapezoidal velocity profile in joint space.

    We will:
    1. Find the largest joint displacement D.
    2. Use trapezoidal profile to compute total time and assign times to each waypoint.
    3. Velocities and accelerations approximated via finite differences.

    Returns:
        times: list of rospy.Duration for each waypoint
        joint_positions_over_time: list of joint positions at each step
        joint_velocities_over_time: list of joint velocities at each step (approx)
        joint_accelerations_over_time: list of joint accelerations at each step (approx)
    """
    point1 = np.array(point1)
    point2 = np.array(point2)
    diff = point2 - point1
    D = np.max(np.abs(diff))  # largest angle displacement defines timing

    # Handle no movement case
    if D < 1e-6:
        joint_positions = np.tile(point1, (steps, 1))
        times = [rospy.Duration(0) for _ in range(steps)]
        velocities = np.zeros((steps, len(point1)))
        accelerations = np.zeros((steps, len(point1)))
        return times, joint_positions, velocities, accelerations

    # Trapezoidal velocity profile timing
    # Check if we can reach joint_vel or if we have a triangular profile
    D_acc = (joint_vel**2) / (2 * joint_acc)

    if D < 2 * D_acc:
        # Triangular profile
        total_time = 2 * np.sqrt(D / joint_acc)
        # Create a normalized distance array
        distances = np.linspace(0, D, steps)
        times_float = []
        for dist in distances:
            if dist <= D / 2:
                # accelerate phase
                t = np.sqrt((2 * dist) / joint_acc)
            else:
                # decelerate phase
                t = total_time - np.sqrt(2 * (D - dist) / joint_acc)
            times_float.append(t)
    else:
        # Trapezoidal profile
        T_a = joint_vel / joint_acc
        D_a = D_acc  # distance covered during accel or decel
        D_c = D - 2 * D_a
        T_c = D_c / joint_vel
        total_time = 2 * T_a + T_c

        distances = np.linspace(0, D, steps)
        times_float = []
        for dist in distances:
            if dist <= D_a:
                # accelerate phase
                t = np.sqrt(2 * dist / joint_acc)
            elif dist <= (D_a + D_c):
                # cruise phase
                t = T_a + (dist - D_a) / joint_vel
            else:
                # decelerate phase
                dist_end = D - dist
                t = total_time - np.sqrt(2 * dist_end / joint_acc)
            times_float.append(t)

    # Convert times_float to rospy.Duration
    times = [rospy.Duration(t) for t in times_float]

    # Interpolate joint positions linearly between point1 and point2
    # Create a fraction array for each step and apply it to interpolate each joint
    fraction = np.linspace(0, 1, steps)
    joint_positions = np.array([point1 + frac * diff for frac in fraction])  # steps x num_joints

    # Approximate velocities and accelerations by finite differences
    # We'll do it joint-by-joint
    times_s = np.array(times_float)
    joint_velocities = np.gradient(joint_positions, times_s, axis=0)  # numerical velocity
    joint_accelerations = np.gradient(joint_velocities, times_s, axis=0)  # numerical acceleration

    return times, joint_positions, joint_velocities, joint_accelerations
