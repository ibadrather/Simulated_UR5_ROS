#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from collections import deque
import matplotlib.pyplot as plt
import time
from typing import Dict, Deque, List, Optional, Tuple
import threading

import PyKDL as KDL
import kdl_parser_py.urdf

# Constants
JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

BASE_LINK = "base_link"
END_EFFECTOR_LINK = "wrist_3_link"


class JointStatePlotter:
    """
    A ROS node that subscribes to joint states, computes forward kinematics to obtain
    the end-effector's pose, buffers joint positions and end-effector positions, and
    plots the joint positions along with the end-effector's trajectory in X, Y, Z over time.
    """

    def __init__(self, buffer_size: int = 1000) -> None:
        """
        Initializes the JointStatePlotter by setting up ROS subscribers and publishers,
        initializing KDL solvers, and preparing data buffers for plotting.

        Args:
            buffer_size (int, optional): Maximum number of data points to buffer for each joint and end-effector position. Defaults to 1000.
        """
        self.buffer_size: int = buffer_size
        self.joint_buffers: Dict[str, Deque[float]] = {joint: deque(maxlen=self.buffer_size) for joint in JOINT_NAMES}

        # Buffers for end-effector positions
        self.ee_x: Deque[float] = deque(maxlen=self.buffer_size)
        self.ee_y: Deque[float] = deque(maxlen=self.buffer_size)
        self.ee_z: Deque[float] = deque(maxlen=self.buffer_size)

        # Buffer for timestamps
        self.timestamps: Deque[float] = deque(maxlen=self.buffer_size)

        self.start_time: float = time.time()

        # Thread lock to prevent data inconsistency during plotting
        self.lock = threading.Lock()

        # Initialize KDL solvers
        self.kdl_chain: Optional[KDL.Chain] = None
        self.fk_solver: Optional[KDL.ChainFkSolverPos_recursive] = None
        self.ik_solver: Optional[KDL.ChainIkSolverPos_LMA] = None
        self.initialize_kdl_solvers()

        # Initialize ROS subscriber for joint states
        rospy.Subscriber("/joint_states", JointState, self.joint_state_callback)

        rospy.loginfo("JointStatePlotter initialized successfully.")

    def initialize_kdl_solvers(self) -> None:
        """
        Initializes the KDL chain and associated forward and inverse kinematics solvers
        based on the robot's URDF.
        """
        robot_description: str = rospy.get_param("robot_description", "")
        if not robot_description:
            rospy.logerr("Robot description parameter 'robot_description' is empty.")
            rospy.signal_shutdown("Missing robot_description parameter.")
            return

        ok, tree = kdl_parser_py.urdf.treeFromString(robot_description)
        if not ok:
            rospy.logerr("Failed to parse URDF for KDL solvers.")
            rospy.signal_shutdown("URDF Parsing Failure.")
            return

        self.kdl_chain = tree.getChain(BASE_LINK, END_EFFECTOR_LINK)
        if self.kdl_chain.getNrOfJoints() != len(JOINT_NAMES):
            rospy.logwarn(
                f"Number of joints in KDL chain ({self.kdl_chain.getNrOfJoints()}) does not match expected ({len(JOINT_NAMES)})."
            )

        self.fk_solver = KDL.ChainFkSolverPos_recursive(self.kdl_chain)
        self.ik_solver = KDL.ChainIkSolverPos_LMA(self.kdl_chain)

        rospy.loginfo("KDL chain and solvers initialized successfully.")

    def joint_state_callback(self, msg: JointState) -> None:
        """
        Callback function for handling incoming JointState messages. Updates joint position buffers,
        computes the end-effector's pose using forward kinematics, and updates end-effector position buffers.

        Args:
            msg (JointState): Incoming joint state message.
        """
        if not self.kdl_chain or not self.fk_solver:
            rospy.logerr("KDL chain or forward kinematics solver not initialized.")
            return

        with self.lock:
            # Create a dictionary mapping joint names to positions
            joint_position_dict = dict(zip(msg.name, msg.position))

            # Update joint position buffers
            for joint_name in JOINT_NAMES:
                if joint_name in joint_position_dict:
                    position = joint_position_dict[joint_name]
                    self.joint_buffers[joint_name].append(position)
                else:
                    rospy.logwarn(f"Joint '{joint_name}' not found in JointState message.")
                    self.joint_buffers[joint_name].append(0.0)  # Assign default value

            # Prepare KDL joint array
            joint_array = KDL.JntArray(len(JOINT_NAMES))
            for i, joint_name in enumerate(JOINT_NAMES):
                joint_array[i] = self.joint_buffers[joint_name][-1]

            # Compute forward kinematics
            end_effector_frame = KDL.Frame()
            fk_result = self.fk_solver.JntToCart(joint_array, end_effector_frame)
            if fk_result >= 0:
                position = end_effector_frame.p  # PyKDL.Vector

                # Append to separate x, y, z buffers
                self.ee_x.append(position.x())
                self.ee_y.append(position.y())
                self.ee_z.append(position.z())

                # Append current timestamp
                current_time = time.time() - self.start_time
                self.timestamps.append(current_time)

                rospy.logdebug(
                    f"End-Effector Position: x={position.x():.3f}, y={position.y():.3f}, z={position.z():.3f}"
                )
            else:
                rospy.logerr("Forward Kinematics computation failed.")

    def save_and_plot(self, plot_duration: int = 10) -> None:
        """
        Saves the buffered joint positions and end-effector trajectories to plots.

        Args:
            plot_duration (int, optional): Duration in seconds to determine how much data to plot. Defaults to 10.
        """
        with self.lock:
            # Convert deques to lists
            timestamps = list(self.timestamps)
            ee_x = list(self.ee_x)
            ee_y = list(self.ee_y)
            ee_z = list(self.ee_z)

            # Ensure all buffers have the same length
            min_length = min(
                len(timestamps), min(len(buf) for buf in self.joint_buffers.values()), len(ee_x), len(ee_y), len(ee_z)
            )

            if min_length == 0:
                rospy.logwarn("No data available to plot.")
                return

            # Trim all data to the minimum length
            timestamps = timestamps[-min_length:]
            ee_x = ee_x[-min_length:]
            ee_y = ee_y[-min_length:]
            ee_z = ee_z[-min_length:]
            joint_buffers_trimmed = {joint: list(buf)[-min_length:] for joint, buf in self.joint_buffers.items()}

        # Plot Joint Positions
        plt.figure(figsize=(15, 10))

        # Subplot for Joint Positions
        plt.subplot(2, 1, 1)
        for joint_name, buffer in joint_buffers_trimmed.items():
            plt.plot(timestamps, buffer, label=joint_name)
        plt.title("Joint Positions Over Time")
        plt.xlabel("Time (s)")
        plt.ylabel("Position (rad)")
        plt.legend()
        plt.grid(True)

        # Subplot for End-Effector Positions Over Time
        plt.subplot(2, 1, 2)
        if ee_x:
            plt.plot(timestamps, ee_x, label="X Position")
            plt.plot(timestamps, ee_y, label="Y Position")
            plt.plot(timestamps, ee_z, label="Z Position")
            plt.title("End-Effector Positions Over Time")
            plt.xlabel("Time (s)")
            plt.ylabel("Position (meters)")
            plt.legend()
            plt.grid(True)
        else:
            plt.title("End-Effector Positions Over Time")
            plt.text(0.5, 0.5, "No Data Available", horizontalalignment="center", verticalalignment="center")
            plt.axis("off")

        plt.tight_layout()
        plt.savefig("joint_and_ee_positions_over_time.png", dpi=300)
        rospy.loginfo("Plots saved as 'joint_and_ee_positions_over_time.png'.")
        plt.show()


def main() -> None:
    """
    Main function to initialize the JointStatePlotter node, run it for a specified duration,
    and then save and plot the joint positions and end-effector trajectories.
    """
    rospy.init_node("joint_state_plotter_node", anonymous=True)
    # buffer_size = rospy.get_param("~buffer_size", 1000)
    # plot_duration = rospy.get_param("~plot_duration", 15)  # Duration in seconds to run the node

    buffer_size = 10000
    plot_duration = 30

    plotter = JointStatePlotter(buffer_size=buffer_size)
    rate = rospy.Rate(50)  # 50 Hz
    start_time = time.time()

    rospy.loginfo(f"JointStatePlotter running for {plot_duration} seconds...")

    try:
        while not rospy.is_shutdown() and (time.time() - start_time < plot_duration):
            rate.sleep()
    except rospy.ROSInterruptException:
        rospy.loginfo("JointStatePlotter interrupted.")

    rospy.loginfo("Saving and plotting data...")
    plotter.save_and_plot()


if __name__ == "__main__":
    main()
