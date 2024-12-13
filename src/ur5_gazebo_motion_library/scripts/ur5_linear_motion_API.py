#!/usr/bin/env python3
import os
import sys
from typing import List
from typing import Optional
from typing import Tuple

import kdl_parser_py.urdf
import rospy
from geometry_msgs.msg import Pose
import PyKDL as KDL
from PyKDL import ChainFkSolverPos_recursive
from PyKDL import ChainIkSolverPos_LMA
from PyKDL import Frame
from PyKDL import JntArray
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from utils import JOINT_NAMES, create_pose
from cartesian_space import (
    parametrize_trajectory,
    pose_to_kdl_frame,
    interpolate_poses,
)


class UR5LinearMotionAPI:
    """
    A ROS API for controlling UR5 robot linear motions using Cartesian trajectories.
    """

    def __init__(self) -> None:
        """
        Initializes the UR5LinearMotionAPI class by setting up ROS subscribers and publishers,
        initializing KDL solvers, and setting up joint position storage.
        """
        # Initialize ROS parameters
        robot_description: str = rospy.get_param("robot_description")
        base_link: str = "base_link"
        end_effector_link: str = "wrist_3_link"

        # Initialize KDL chain and solvers
        self.kdl_chain, self.fk_solver, self.ik_solver, self.num_joints = self.initialize_kdl_solvers(
            robot_description, base_link, end_effector_link
        )
        if not self.kdl_chain:
            rospy.logerr("Failed to initialize KDL chain. Shutting down node.")
            rospy.signal_shutdown("KDL Initialization Failure")
            return

        # Initialize joint positions
        self.joint_positions: JntArray = JntArray(self.num_joints)

        # Initialize ROS subscriber for joint states
        self.joint_state_subscriber: rospy.Subscriber = rospy.Subscriber(
            "/joint_states", JointState, self.joint_states_callback
        )

        # Initialize ROS publisher for joint trajectories
        self.trajectory_publisher: rospy.Publisher = rospy.Publisher(
            "/eff_joint_traj_controller/command",
            JointTrajectory,
            queue_size=10,
        )

        rospy.loginfo("UR5LinearMotionAPI initialized successfully.")

    def initialize_kdl_solvers(self, robot_description: str, base_link: str, end_effector_link: str):
        """
        Initializes the KDL chain and associated forward and inverse kinematics solvers.

        Args:
            robot_description (str): URDF string describing the robot.
            base_link (str): Name of the base link.
            end_effector_link (str): Name of the end effector link.

        Returns:
            Tuple containing:
                - kdl_chain (Optional[kdl_parser_py.urdf.Chain]): The KDL chain.
                - fk_solver (Optional[ChainFkSolverPos_recursive]): Forward kinematics solver.
                - ik_solver (Optional[ChainIkSolverPos_LMA]): Inverse kinematics solver.
                - num_joints (int): Number of joints in the chain.
        """
        ok, tree = kdl_parser_py.urdf.treeFromString(robot_description)
        if not ok:
            rospy.logerr("Failed to parse URDF for KDL solvers.")
            return None, None, None, 0

        kdl_chain = tree.getChain(base_link, end_effector_link)
        if kdl_chain.getNrOfJoints() != len(JOINT_NAMES):
            rospy.logwarn(
                f"Number of joints in KDL chain ({kdl_chain.getNrOfJoints()}) does not match expected ({len(JOINT_NAMES)})."
            )

        fk_solver = ChainFkSolverPos_recursive(kdl_chain)
        ik_solver = ChainIkSolverPos_LMA(kdl_chain)
        num_joints = kdl_chain.getNrOfJoints()

        rospy.loginfo("KDL chain and solvers initialized successfully.")
        return kdl_chain, fk_solver, ik_solver, num_joints

    def joint_states_callback(self, msg: JointState) -> None:
        """
        Callback function for updating current joint positions from JointState messages.

        Args:
            msg (JointState): Incoming joint state message.
        """
        if len(msg.position) < self.num_joints:
            rospy.logwarn(
                f"Received JointState with insufficient positions: expected {self.num_joints}, got {len(msg.position)}."
            )
            return

        for i in range(self.num_joints):
            self.joint_positions[i] = msg.position[i]

        # Compute forward kinematics to get end-effector frame
        current_frame = KDL.Frame()
        fk_result = self.fk_solver.JntToCart(self.joint_positions, current_frame)

        if fk_result >= 0:
            position = current_frame.p  # PyKDL.Vector containing x, y, z
            orientation = current_frame.M.GetQuaternion()  # Tuple (x, y, z, w)

            # Log the end-effector position and orientation
            # rospy.loginfo(
            #     f"End-Effector Position: x={position.x():.3f}, y={position.y():.3f}, z={position.z():.3f}"
            # )
            # rospy.loginfo(
            #     f"End-Effector Orientation: x={orientation[0]:.3f}, y={orientation[1]:.3f}, "
            #     f"z={orientation[2]:.3f}, w={orientation[3]:.3f}"
            # )
        else:
            rospy.logerr("Failed to compute forward kinematics for current joint positions.")

    def get_current_joint_positions(self) -> List[float]:
        """
        Retrieves the current joint positions.

        Returns:
            List[float]: Current positions of all joints.
        """
        return [self.joint_positions[i] for i in range(self.num_joints)]

    def calculate_joint_trajectory(self, waypoints: List[Pose]) -> Optional[List[List[float]]]:
        """
        Calculates the joint trajectory by solving inverse kinematics for each Cartesian waypoint.

        Args:
            waypoints (List[Pose]): List of Cartesian poses representing waypoints.

        Returns:
            Optional[List[List[float]]]: Joint configurations for each waypoint, or None if IK fails.
        """
        joint_trajectory: List[List[float]] = []
        initial_joints: JntArray = JntArray(self.num_joints)

        for idx, waypoint in enumerate(waypoints):
            frame: Frame = pose_to_kdl_frame(waypoint)
            waypoint_joints: JntArray = JntArray(self.num_joints)

            ik_result: int = self.ik_solver.CartToJnt(initial_joints, frame, waypoint_joints)
            if ik_result < 0:
                rospy.logerr(f"Inverse Kinematics failed for waypoint {idx}.")
                return None

            joint_trajectory.append([waypoint_joints[i] for i in range(self.num_joints)])
            initial_joints = waypoint_joints  # Update for the next IK solve

        rospy.loginfo("Joint trajectory calculated successfully via IK.")
        return joint_trajectory

    def generate_cartesian_trajectory(
        self,
        start_pose: Pose,
        end_pose: Pose,
        linear_vel: float,
        linear_acc: float,
        steps: int = 200,
    ) -> Optional[JointTrajectory]:
        """
        Generates a Cartesian trajectory from start_pose to end_pose, parameterizes it, and converts to joint space.

        Args:
            start_pose (Pose): Starting Cartesian pose.
            end_pose (Pose): Ending Cartesian pose.
            linear_vel (float): Desired linear velocity (m/s).
            linear_acc (float): Desired linear acceleration (m/s²).
            steps (int, optional): Number of interpolation steps. Defaults to 200.

        Returns:
            Optional[JointTrajectory]: The generated joint trajectory message, or None if generation fails.
        """
        # Interpolate Cartesian waypoints
        waypoints: List[Pose] = interpolate_poses(start_pose, end_pose, steps)
        rospy.loginfo(f"Interpolated {len(waypoints)} Cartesian waypoints.")

        # Solve IK for each waypoint to get joint configurations
        joint_trajectory: Optional[List[List[float]]] = self.calculate_joint_trajectory(waypoints)
        if joint_trajectory is None:
            rospy.logerr("Failed to calculate joint trajectory via IK.")
            return None

        # Parameterize the trajectory with timing, velocities, and accelerations
        times, velocities, accelerations = parametrize_trajectory(waypoints, linear_vel, linear_acc)
        rospy.loginfo("Trajectory parameterization completed.")

        # Assign times, velocities, accelerations to each JointTrajectoryPoint
        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = JOINT_NAMES
        for i, joints in enumerate(joint_trajectory):
            point = JointTrajectoryPoint()
            point.positions = joints
            # Here we assign the same velocity/acceleration across all joints for simplicity
            point.velocities = [velocities[i]] * len(joints)
            point.accelerations = [accelerations[i]] * len(joints)
            point.time_from_start = times[i]
            trajectory_msg.points.append(point)

        rospy.loginfo("JointTrajectory message constructed successfully.")
        return trajectory_msg

    def execute_trajectory(self, trajectory_msg: JointTrajectory) -> None:
        """
        Publishes the given joint trajectory to the robot controller.

        Args:
            trajectory_msg (JointTrajectory): The joint trajectory message to publish.
        """
        self.trajectory_publisher.publish(trajectory_msg)
        rospy.loginfo("Joint space trajectory published to the controller.")


def main() -> None:
    """
    Main function to initialize the UR5LinearMotionAPI node, generate a Cartesian trajectory,
    and execute it.
    """
    rospy.init_node("ur5_linear_motion_API_node")

    # Create the API object
    ur5_api = UR5LinearMotionAPI()

    # Wait for the first joint states message to be received
    rospy.loginfo("Waiting for initial joint states...")
    try:
        rospy.wait_for_message("/joint_states", JointState, timeout=5.0)
    except rospy.ROSException:
        rospy.logerr("Timeout while waiting for /joint_states. Shutting down node.")
        rospy.signal_shutdown("No joint states received.")
        return

    rospy.sleep(1.0)  # Additional sleep to ensure callback has been processed

    # Retrieve current joint positions
    current_joints: List[float] = ur5_api.get_current_joint_positions()
    rospy.loginfo(f"Current joint positions: {current_joints}")

    # Define a target Cartesian pose (example)
    # Here we move the end effector in the Y-axis by 1.2 meters
    start_pose: Pose = create_pose(0.3, 0.6, 0.1, 1.0, 0.0, 0.0, 0.0)
    end_pose: Pose = create_pose(0.3, -0.6, 0.1, 1.0, 0.0, 0.0, 0.0)
    rospy.loginfo(f"Start Pose: {start_pose}")
    rospy.loginfo(f"End Pose: {end_pose}")

    # Define desired linear velocity and acceleration
    linear_vel: float = 5  # m/s
    linear_acc: float = 1.0  # m/s²

    # Generate the Cartesian trajectory
    trajectory_msg: Optional[JointTrajectory] = ur5_api.generate_cartesian_trajectory(
        start_pose, end_pose, linear_vel, linear_acc, steps=200
    )

    if trajectory_msg is not None:
        # Execute the trajectory
        ur5_api.execute_trajectory(trajectory_msg)
    else:
        rospy.logerr("Failed to generate or execute Cartesian trajectory.")


if __name__ == "__main__":
    try:
        main()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("UR5LinearMotionAPI node terminated.")
