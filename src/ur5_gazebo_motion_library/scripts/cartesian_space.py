import rospy
from geometry_msgs.msg import Pose
import numpy as np
from typing import List
import PyKDL as KDL


def pose_to_kdl_frame(pose: Pose) -> KDL.Frame:
    return KDL.Frame(
        KDL.Rotation.Quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w),
        KDL.Vector(pose.position.x, pose.position.y, pose.position.z),
    )


def interpolate_poses(start: Pose, end: Pose, steps: int) -> List[Pose]:
    waypoints = []
    for t in np.linspace(0, 1, steps):
        interp_pose = Pose()
        interp_pose.position.x = (1 - t) * start.position.x + t * end.position.x
        interp_pose.position.y = (1 - t) * start.position.y + t * end.position.y
        interp_pose.position.z = (1 - t) * start.position.z + t * end.position.z
        interp_pose.orientation = start.orientation  # Keeping orientation constant for simplicity
        waypoints.append(interp_pose)
    return waypoints


def distance_between_poses(p1: Pose, p2: Pose) -> float:
    dx = p2.position.x - p1.position.x
    dy = p2.position.y - p1.position.y
    dz = p2.position.z - p1.position.z
    return np.sqrt(dx * dx + dy * dy + dz * dz)


def parametrize_trajectory(waypoints: List[Pose], linear_vel: float, linear_acc: float):
    # Same parameterization logic as previously shown
    start_pose = waypoints[0]
    end_pose = waypoints[-1]
    D = distance_between_poses(start_pose, end_pose)

    if D < 1e-6:
        times = [rospy.Duration(0) for _ in waypoints]
        velocities = [0.0 for _ in waypoints]
        accelerations = [0.0 for _ in waypoints]
        return times, velocities, accelerations

    D_acc = (linear_vel**2) / (2 * linear_acc)

    if D < 2 * D_acc:
        # Triangle profile
        total_time = np.sqrt(D / linear_acc) * 2
        distances = np.linspace(0, D, len(waypoints))
        times = []
        for dist in distances:
            half_T = total_time / 2
            if dist <= D / 2:
                t = np.sqrt((2 * dist) / linear_acc)
            else:
                t = total_time - np.sqrt(2 * (D - dist) / linear_acc)
            times.append(rospy.Duration(t))
        times_float = [t.to_sec() for t in times]
        velocities = np.gradient(distances, times_float)
        accelerations = np.gradient(velocities, times_float)
        return times, velocities, accelerations
    else:
        # Trapezoidal profile
        T_a = linear_vel / linear_acc
        D_a = D_acc
        D_c = D - 2 * D_a
        if D_c < 0:
            D_c = 0
        T_c = D_c / linear_vel
        total_time = 2 * T_a + T_c
        distances = np.linspace(0, D, len(waypoints))
        times = []
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
        velocities = np.gradient(distances, times_float)
        accelerations = np.gradient(velocities, times_float)
        return times, velocities, accelerations
