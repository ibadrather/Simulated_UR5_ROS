#!/usr/bin/env python3

import rospy
import kdl_parser_py.urdf
from PyKDL import ChainFkSolverPos_recursive, ChainIkSolverPos_LMA, JntArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from typing import List

import os
import sys


sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from utils import JOINT_NAMES
from joint_space import parametrize_joint_trajectory


class UR5JointMotionAPI:
    def __init__(self):
        # Initialize ROS components
        self.num_joints = len(JOINT_NAMES)
        self.joint_positions = JntArray(self.num_joints)

        rospy.Subscriber("/joint_states", JointState, self.joint_states_callback)

        self.trajectory_pub = rospy.Publisher("/eff_joint_traj_controller/command", JointTrajectory, queue_size=10)

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

    def generate_joint_space_trajectory(
        self, point1: List[float], point2: List[float], joint_vel: float, joint_acc: float, steps: int = 100
    ) -> JointTrajectory:
        """
        Generate a joint space trajectory that moves from point1 to point2 with given velocity and acceleration constraints.
        """
        if len(point1) != self.num_joints or len(point2) != self.num_joints:
            rospy.logerr("Point1 and Point2 must have length equal to number of joints.")
            return None

        times, joint_positions, joint_velocities, joint_accelerations = parametrize_joint_trajectory(
            point1, point2, steps, joint_vel, joint_acc
        )

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
    rospy.init_node("ur5_joint_motion_API_node")

    ur5_api = UR5JointMotionAPI()

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

    trajectory_msg = ur5_api.generate_joint_space_trajectory(
        current_joints, target_joints, joint_vel, joint_acc, steps=100
    )

    if trajectory_msg is not None:
        ur5_api.execute_trajectory(trajectory_msg)
    else:
        rospy.logerr("Failed to generate joint space trajectory.")


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
