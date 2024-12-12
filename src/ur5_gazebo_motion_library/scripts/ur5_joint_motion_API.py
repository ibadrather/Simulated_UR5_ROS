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

JOINT_NAMES = [
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
]


def parametrize_joint_trajectory(point1: List[float], point2: List[float], steps: int, joint_vel: float, joint_acc: float):
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
    D_acc = (joint_vel**2) / (2*joint_acc)

    if D < 2*D_acc:
        # Triangular profile
        total_time = 2*np.sqrt(D/joint_acc)
        # Create a normalized distance array
        distances = np.linspace(0, D, steps)
        times_float = []
        for dist in distances:
            if dist <= D/2:
                # accelerate phase
                t = np.sqrt((2*dist)/joint_acc)
            else:
                # decelerate phase
                t = total_time - np.sqrt(2*(D-dist)/joint_acc)
            times_float.append(t)
    else:
        # Trapezoidal profile
        T_a = joint_vel / joint_acc
        D_a = D_acc  # distance covered during accel or decel
        D_c = D - 2*D_a
        T_c = D_c / joint_vel
        total_time = 2*T_a + T_c

        distances = np.linspace(0, D, steps)
        times_float = []
        for dist in distances:
            if dist <= D_a:
                # accelerate phase
                t = np.sqrt(2*dist/joint_acc)
            elif dist <= (D_a + D_c):
                # cruise phase
                t = T_a + (dist - D_a)/joint_vel
            else:
                # decelerate phase
                dist_end = D - dist
                t = total_time - np.sqrt(2*dist_end/joint_acc)
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


class UR5MotionAPI:
    def __init__(self):
        # Initialize ROS components
        robot_description = rospy.get_param("robot_description")
        base_link = "base_link"
        end_effector_link = "wrist_3_link"

        kdl_chain, fk_solver, ik_solver, num_joints = self.initialize_kdl_solvers(
            robot_description, base_link, end_effector_link
        )
        if not kdl_chain:
            rospy.logerr("Failed to initialize KDL chain.")
            return

        self.kdl_chain = kdl_chain
        self.fk_solver = fk_solver
        self.ik_solver = ik_solver
        self.num_joints = num_joints
        self.joint_positions = JntArray(num_joints)

        rospy.Subscriber("/joint_states", JointState, self.joint_states_callback)

        self.trajectory_pub = rospy.Publisher(
            "/eff_joint_traj_controller/command", JointTrajectory, queue_size=10
        )

    def initialize_kdl_solvers(self, robot_description: str, base_link: str, end_effector_link: str):
        ok, tree = kdl_parser_py.urdf.treeFromString(robot_description)
        if not ok:
            rospy.logerr("Failed to parse URDF")
            return None, None, None, None
        kdl_chain = tree.getChain(base_link, end_effector_link)
        fk_solver = ChainFkSolverPos_recursive(kdl_chain)
        ik_solver = ChainIkSolverPos_LMA(kdl_chain)
        num_joints = kdl_chain.getNrOfJoints()
        return kdl_chain, fk_solver, ik_solver, num_joints

    def joint_states_callback(self, msg: JointState):
        if len(msg.position) >= self.num_joints:
            for i in range(self.num_joints):
                self.joint_positions[i] = msg.position[i]

    def get_current_joint_positions(self):
        return [self.joint_positions[i] for i in range(self.num_joints)]

    def generate_joint_space_trajectory(self, point1: List[float], point2: List[float], joint_vel: float, joint_acc: float, steps: int = 100) -> JointTrajectory:
        """
        Generate a joint space trajectory that moves from point1 to point2 with given velocity and acceleration constraints.
        """
        if len(point1) != self.num_joints or len(point2) != self.num_joints:
            rospy.logerr("Point1 and Point2 must have length equal to number of joints.")
            return None

        times, joint_positions, joint_velocities, joint_accelerations = parametrize_joint_trajectory(point1, point2, steps, joint_vel, joint_acc)

        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = JOINT_NAMES

        for i in range(steps):
            point = JointTrajectoryPoint()
            point.positions = joint_positions[i, :].tolist()
            point.velocities = joint_velocities[i, :].tolist()
            point.accelerations = joint_accelerations[i, :].tolist()
            point.time_from_start = times[i]
            trajectory_msg.points.append(point)

        return trajectory_msg

    def execute_trajectory(self, trajectory_msg: JointTrajectory):
        """
        Publish the given trajectory to the robot controller.
        """
        self.trajectory_pub.publish(trajectory_msg)
        rospy.loginfo("Joint Space Trajectory published.")


def main():
    rospy.init_node("ur5_joint_space_motion_node")

    ur5_api = UR5MotionAPI()

    # Wait for joint states
    rospy.sleep(1.0)

    # Current joint positions
    current_joints = ur5_api.get_current_joint_positions()

    # Define a target joint configuration (for example)
    # Here we just move each joint by a small offset
    target_joints = [cj + 0.5 for cj in current_joints]

    # Define desired joint velocity and acceleration
    joint_vel = 0.5  # rad/s
    joint_acc = 0.2  # rad/s^2

    trajectory_msg = ur5_api.generate_joint_space_trajectory(current_joints, target_joints, joint_vel, joint_acc, steps=100)

    if trajectory_msg is not None:
        ur5_api.execute_trajectory(trajectory_msg)
    else:
        rospy.logerr("Failed to generate joint space trajectory.")


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
