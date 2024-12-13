#!/usr/bin/env python3
import os
import sys
from typing import List
from typing import Optional

import rospy
from PyKDL import JntArray
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from utils import JOINT_NAMES
from joint_space import parametrize_joint_trajectory


class UR5JointMotionAPI:
    """
    A ROS API for controlling UR5 robot joint motions using joint space trajectories.
    """

    def __init__(self) -> None:
        """
        Initializes the UR5JointMotionAPI class by setting up ROS subscribers and publishers,
        and initializing joint positions.
        """
        # Initialize ROS parameters
        self.num_joints: int = len(JOINT_NAMES)
        self.joint_positions: JntArray = JntArray(self.num_joints)

        # Initialize ROS subscriber for joint states
        self.joint_state_subscriber = rospy.Subscriber("/joint_states", JointState, self.joint_states_callback)

        # Initialize ROS publisher for joint trajectories
        self.trajectory_publisher: rospy.Publisher = rospy.Publisher(
            "/eff_joint_traj_controller/command",
            JointTrajectory,
            queue_size=10,
        )

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

    def get_current_joint_positions(self) -> List[float]:
        """
        Retrieves the current joint positions.

        Returns:
            List[float]: Current positions of all joints.
        """
        return [self.joint_positions[i] for i in range(self.num_joints)]

    def generate_joint_space_trajectory(
        self,
        point1: List[float],
        point2: List[float],
        joint_vel: float,
        joint_acc: float,
        steps: int = 100,
    ) -> Optional[JointTrajectory]:
        """
        Generates a joint space trajectory from point1 to point2 with specified velocity and acceleration constraints.

        Args:
            point1 (List[float]): Starting joint configuration.
            point2 (List[float]): Ending joint configuration.
            joint_vel (float): Maximum joint velocity (rad/s).
            joint_acc (float): Maximum joint acceleration (rad/s²).
            steps (int, optional): Number of interpolation steps. Defaults to 100.

        Returns:
            Optional[JointTrajectory]: The generated joint trajectory message, or None if generation fails.
        """
        if len(point1) != self.num_joints or len(point2) != self.num_joints:
            rospy.logerr("Point1 and Point2 must have lengths equal to the number of joints.")
            return None

        trajectory_times, joint_positions, joint_velocities, joint_accelerations = parametrize_joint_trajectory(
            point1, point2, steps, joint_vel, joint_acc
        )

        if not trajectory_times:
            rospy.logerr("Failed to parameterize joint trajectory.")
            return None

        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = JOINT_NAMES

        for i in range(steps):
            point = JointTrajectoryPoint()
            point.positions = joint_positions[i, :].tolist()
            point.velocities = joint_velocities[i, :].tolist()
            point.accelerations = joint_accelerations[i, :].tolist()
            point.time_from_start = trajectory_times[i]
            trajectory_msg.points.append(point)

        rospy.loginfo("Joint space trajectory generated successfully.")
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
    Main function to initialize the UR5JointMotionAPI node, generate a trajectory,
    and execute it.
    """
    rospy.init_node("ur5_joint_motion_API_node")

    ur5_api = UR5JointMotionAPI()

    # Wait for the first joint states message to be received
    rospy.loginfo("Waiting for initial joint states...")
    rospy.wait_for_message("/joint_states", JointState, timeout=5.0)
    rospy.sleep(1.0)

    # Retrieve current joint positions
    current_joints = ur5_api.get_current_joint_positions()
    rospy.loginfo(f"Current joint positions: {current_joints}")

    # Define a target joint configuration (example: move each joint by 0.5 radians)
    target_joints = [cj + 0.5 for cj in current_joints]
    rospy.loginfo(f"Target joint positions: {target_joints}")

    # Define desired joint velocity and acceleration
    joint_vel = 0.5  # rad/s
    joint_acc = 0.2  # rad/s²

    # Generate the joint space trajectory
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
        rospy.loginfo("UR5JointMotionAPI node terminated.")
