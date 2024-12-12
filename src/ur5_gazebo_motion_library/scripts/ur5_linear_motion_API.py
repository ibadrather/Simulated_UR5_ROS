#!/usr/bin/env python3

import rospy
import kdl_parser_py.urdf
from PyKDL import ChainFkSolverPos_recursive, ChainIkSolverPos_LMA, JntArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from typing import List

import os
import sys


sys.path.append(os.path.dirname(os.path.abspath(__file__)))


from utils import JOINT_NAMES, create_pose
from cartesian_space import (
    parametrize_trajectory,
    pose_to_kdl_frame,
    interpolate_poses,
)


class UR5LinearMotionAPI:
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

        # You might keep a publisher for executing trajectories here
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

    def calculate_joint_trajectory(self, waypoints: List[Pose]) -> List[List[float]]:
        joint_trajectory = []
        initial_joints = JntArray(self.num_joints)
        for waypoint in waypoints:
            frame = pose_to_kdl_frame(waypoint)
            waypoint_joints = JntArray(self.num_joints)
            if self.ik_solver.CartToJnt(initial_joints, frame, waypoint_joints) < 0:
                rospy.logerr("IK failed for a waypoint")
                return None
            joint_trajectory.append([waypoint_joints[i] for i in range(self.num_joints)])
        return joint_trajectory

    def generate_cartesian_trajectory(
        self, start_pose: Pose, end_pose: Pose, linear_vel: float, linear_acc: float, steps: int = 200
    ) -> JointTrajectory:
        # Interpolate and solve IK
        waypoints = interpolate_poses(start_pose, end_pose, steps)
        joint_trajectory = self.calculate_joint_trajectory(waypoints)
        if not joint_trajectory:
            return None

        # Parametrize trajectory
        times, velocities, accelerations = parametrize_trajectory(waypoints, linear_vel, linear_acc)

        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = JOINT_NAMES

        # Assign times, velocities, accelerations to each JointTrajectoryPoint
        for i, joints in enumerate(joint_trajectory):
            point = JointTrajectoryPoint()
            point.positions = joints
            # Here we assign the same velocity/acceleration across all joints for simplicity
            point.velocities = [velocities[i]] * len(joints)
            point.accelerations = [accelerations[i]] * len(joints)
            point.time_from_start = times[i]
            trajectory_msg.points.append(point)

        return trajectory_msg

    def execute_trajectory(self, trajectory_msg: JointTrajectory):
        """
        Publish the given trajectory to the robot controller.
        """
        self.trajectory_pub.publish(trajectory_msg)
        rospy.loginfo("Trajectory published with given velocity/acceleration constraints.")


def main():
    rospy.init_node("ur5_linear_motion_API_node")

    # Create the API object
    ur5_api = UR5LinearMotionAPI()

    # Define start and end poses and parameters externally
    # In a real scenario, these could come from another script or user input
    start_pose = create_pose(0.3, 0.6, 0.1, 1.0, 0.0, 0.0, 0.0)
    end_pose = create_pose(0.3, -0.6, 0.1, 1.0, 0.0, 0.0, 0.0)
    linear_vel = 0.9  # user-specified linear velocity
    linear_acc = 0.3  # user-specified linear acceleration

    rospy.sleep(1)  # Ensure subscriber is ready and we have joint states

    # Use the API to generate a trajectory
    trajectory_msg = ur5_api.generate_cartesian_trajectory(start_pose, end_pose, linear_vel, linear_acc, steps=200)

    if trajectory_msg is not None:
        # Execute the trajectory
        ur5_api.execute_trajectory(trajectory_msg)
    else:
        rospy.logerr("Failed to generate trajectory.")


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
