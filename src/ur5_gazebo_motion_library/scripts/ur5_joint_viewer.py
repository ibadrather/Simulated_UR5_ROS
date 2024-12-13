#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
from collections import deque
import matplotlib.pyplot as plt
import time
from typing import Dict, Deque, List, Optional, Tuple
import threading
import time
import PyKDL as KDL
import kdl_parser_py.urdf
from datetime import datetime
import csv
import os

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
    the end-effector's pose, buffers joint positions, velocities, accelerations and
    end-effector positions, velocities, accelerations, and plots them over time.
    """

    def __init__(self, buffer_size: int = 1000) -> None:
        """
        Initializes the JointStatePlotter by setting up ROS subscribers and publishers,
        initializing KDL solvers, and preparing data buffers for plotting.

        Args:
            buffer_size (int, optional): Maximum number of data points to buffer for each joint and end-effector position. Defaults to 1000.
        """
        self.buffer_size: int = buffer_size
        self.save_plots = False
        # Buffers for Joint Data
        self.joint_positions: Dict[str, Deque[float]] = {joint: deque(maxlen=self.buffer_size) for joint in JOINT_NAMES}
        self.joint_velocities: Dict[str, Deque[float]] = {
            joint: deque(maxlen=self.buffer_size) for joint in JOINT_NAMES
        }
        self.joint_accelerations: Dict[str, Deque[float]] = {
            joint: deque(maxlen=self.buffer_size) for joint in JOINT_NAMES
        }

        # Buffers for End-Effector positions
        self.ee_x: Deque[float] = deque(maxlen=self.buffer_size)
        self.ee_y: Deque[float] = deque(maxlen=self.buffer_size)
        self.ee_z: Deque[float] = deque(maxlen=self.buffer_size)

        # Buffers for End-Effector velocities
        self.ee_vx: Deque[float] = deque(maxlen=self.buffer_size)
        self.ee_vy: Deque[float] = deque(maxlen=self.buffer_size)
        self.ee_vz: Deque[float] = deque(maxlen=self.buffer_size)

        # Buffers for End-Effector accelerations
        self.ee_ax: Deque[float] = deque(maxlen=self.buffer_size)
        self.ee_ay: Deque[float] = deque(maxlen=self.buffer_size)
        self.ee_az: Deque[float] = deque(maxlen=self.buffer_size)

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

        # Dating Timestamp
        self.naming_timestamp = datetime.now().strftime("%y-%m-%d %a %H:%M:%S")

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

    def compute_derivatives(self, data: Deque[float], times: Deque[float]) -> Optional[Tuple[float, float]]:
        """
        Compute velocity and acceleration given a deque of data and corresponding times.

        Velocity is computed as the finite difference of the last two points.
        Acceleration is computed as the finite difference of the last two velocities.

        Returns:
            (velocity, acceleration) if enough data points exist, otherwise None.
        """
        if len(data) < 2 or len(times) < 2:
            return None

        # Compute velocity
        pos_new = data[-1]
        pos_old = data[-2]
        t_new = times[-1]
        t_old = times[-2]

        dt = t_new - t_old
        if dt <= 0:
            return None

        velocity = (pos_new - pos_old) / dt

        # To compute acceleration, we need at least 3 points
        if len(data) < 3 or len(times) < 3:
            return velocity, None

        # Velocity old (from one step before)
        pos_old2 = data[-3]
        t_old2 = times[-3]
        dt2 = t_old - t_old2
        if dt2 <= 0:
            return velocity, None

        velocity_old = (pos_old - pos_old2) / dt2

        acceleration = (velocity - velocity_old) / dt

        return velocity, acceleration

    def joint_state_callback(self, msg: JointState) -> None:
        """
        Callback function for handling incoming JointState messages. Updates joint position buffers,
        computes joint velocities and accelerations, computes the end-effector's pose and its velocity
        and acceleration, and stores all data in buffers.

        Args:
            msg (JointState): Incoming joint state message.
        """
        if not self.kdl_chain or not self.fk_solver:
            rospy.logerr("KDL chain or forward kinematics solver not initialized.")
            return

        with self.lock:
            # Current time
            current_time = time.time() - self.start_time
            self.timestamps.append(current_time)

            # Create a dictionary mapping joint names to positions, velocities, if available
            joint_position_dict = dict(zip(msg.name, msg.position))
            joint_velocity_dict = dict(zip(msg.name, msg.velocity)) if msg.velocity else {}

            # Update joint data
            for joint_name in JOINT_NAMES:
                position = joint_position_dict.get(joint_name, 0.0)
                self.joint_positions[joint_name].append(position)

                # If velocity data is available directly from the message, use it
                # Otherwise, compute from position differences
                if joint_name in joint_velocity_dict:
                    vel = joint_velocity_dict[joint_name]
                    self.joint_velocities[joint_name].append(vel)

                    # Compute acceleration numerically if possible
                    if len(self.joint_velocities[joint_name]) > 1:
                        v_new = self.joint_velocities[joint_name][-1]
                        v_old = self.joint_velocities[joint_name][-2]
                        t_new = self.timestamps[-1]
                        t_old = self.timestamps[-2] if len(self.timestamps) > 1 else None
                        if t_old is not None:
                            dt = t_new - t_old
                            acc = (v_new - v_old) / dt if dt > 0 else 0.0
                            self.joint_accelerations[joint_name].append(acc)
                        else:
                            self.joint_accelerations[joint_name].append(0.0)
                    else:
                        self.joint_accelerations[joint_name].append(0.0)
                else:
                    # Compute velocity and acceleration from position data
                    derivatives = self.compute_derivatives(self.joint_positions[joint_name], self.timestamps)
                    if derivatives:
                        vel, acc = derivatives
                        if vel is not None:
                            self.joint_velocities[joint_name].append(vel)
                        else:
                            self.joint_velocities[joint_name].append(0.0)
                        if acc is not None:
                            self.joint_accelerations[joint_name].append(acc)
                        else:
                            self.joint_accelerations[joint_name].append(0.0)
                    else:
                        # Not enough data points yet
                        self.joint_velocities[joint_name].append(0.0)
                        self.joint_accelerations[joint_name].append(0.0)

            # Prepare KDL joint array
            joint_array = KDL.JntArray(len(JOINT_NAMES))
            for i, joint_name in enumerate(JOINT_NAMES):
                joint_array[i] = self.joint_positions[joint_name][-1]

            # Compute forward kinematics for end-effector
            end_effector_frame = KDL.Frame()
            fk_result = self.fk_solver.JntToCart(joint_array, end_effector_frame)
            if fk_result >= 0:
                position = end_effector_frame.p  # PyKDL.Vector

                self.ee_x.append(position.x())
                self.ee_y.append(position.y())
                self.ee_z.append(position.z())

                # Compute end-effector velocities and accelerations
                # Velocity and acceleration in x
                deriv_x = self.compute_derivatives(self.ee_x, self.timestamps)
                deriv_y = self.compute_derivatives(self.ee_y, self.timestamps)
                deriv_z = self.compute_derivatives(self.ee_z, self.timestamps)

                # For each dimension, append computed velocities and accelerations
                if deriv_x:
                    vx, ax = deriv_x
                    self.ee_vx.append(vx if vx is not None else 0.0)
                    self.ee_ax.append(ax if ax is not None else 0.0)
                else:
                    self.ee_vx.append(0.0)
                    self.ee_ax.append(0.0)

                if deriv_y:
                    vy, ay = deriv_y
                    self.ee_vy.append(vy if vy is not None else 0.0)
                    self.ee_ay.append(ay if ay is not None else 0.0)
                else:
                    self.ee_vy.append(0.0)
                    self.ee_ay.append(0.0)

                if deriv_z:
                    vz, az = deriv_z
                    self.ee_vz.append(vz if vz is not None else 0.0)
                    self.ee_az.append(az if az is not None else 0.0)
                else:
                    self.ee_vz.append(0.0)
                    self.ee_az.append(0.0)

                rospy.logdebug(
                    f"End-Effector Position: x={position.x():.3f}, y={position.y():.3f}, z={position.z():.3f}"
                )
            else:
                rospy.logerr("Forward Kinematics computation failed.")

    def plot_joint_data(
        self, timestamps: List[float], data_dict: Dict[str, List[float]], title_prefix: str, ylabel: str, filename: str
    ) -> None:
        """
        Create a figure with subplots for each joint, plotting the given data (positions/velocities/accelerations).

        Args:
            timestamps (List[float]): The time array
            data_dict (Dict[str, List[float]]): Dictionary of joint_name -> data array
            title_prefix (str): Title prefix (e.g., "Joint Positions", "Joint Velocities")
            ylabel (str): Y-axis label
            filename (str): Filename to save the figure
        """
        num_joints = len(JOINT_NAMES)
        fig, axes = plt.subplots(num_joints, 1, figsize=(8, 12), sharex=True)
        fig.suptitle(title_prefix, fontsize=16)
        if num_joints == 1:
            axes = [axes]  # Ensure axes is iterable
        for i, joint_name in enumerate(JOINT_NAMES):
            axes[i].plot(timestamps, data_dict[joint_name], label=joint_name)
            axes[i].set_title(joint_name)
            axes[i].set_ylabel(ylabel)
            axes[i].grid(True)
        axes[-1].set_xlabel("Time (s)")

        plt.tight_layout(rect=[0, 0.03, 1, 0.95])
        if self.save_plots:
            plt.savefig(filename + "_" + self.naming_timestamp + ".png", dpi=300)
        rospy.loginfo(f"{title_prefix} saved as '{filename}'.")
        plt.close(fig)

    def plot_ee_data(
        self,
        timestamps: List[float],
        data_x: List[float],
        data_y: List[float],
        data_z: List[float],
        title_prefix: str,
        ylabel: str,
        filename: str,
    ) -> None:
        """
        Create a figure with 3 subplots for end-effector data (x, y, z).

        Args:
            timestamps (List[float]): The time array
            data_x, data_y, data_z (List[float]): The data arrays for x, y, z
            title_prefix (str): Title prefix (e.g., "End-Effector Positions")
            ylabel (str): Y-axis label
            filename (str): Filename to save
        """
        fig, axes = plt.subplots(3, 1, figsize=(8, 10), sharex=True)
        fig.suptitle(title_prefix, fontsize=16)

        axes[0].plot(timestamps, data_x, label="X")
        axes[0].set_title("X")
        axes[0].set_ylabel(ylabel)
        axes[0].grid(True)

        axes[1].plot(timestamps, data_y, label="Y")
        axes[1].set_title("Y")
        axes[1].set_ylabel(ylabel)
        axes[1].grid(True)

        axes[2].plot(timestamps, data_z, label="Z")
        axes[2].set_title("Z")
        axes[2].set_ylabel(ylabel)
        axes[2].set_xlabel("Time (s)")
        axes[2].grid(True)

        plt.tight_layout(rect=[0, 0.03, 1, 0.95])
        if self.save_plots:
            plt.savefig(filename + "_" + self.naming_timestamp + ".png", dpi=300)
        rospy.loginfo(f"{title_prefix} saved as '{filename}'.")
        plt.close(fig)

    def save_and_plot(self) -> None:
        """
        Saves the buffered joint positions, velocities, accelerations and
        end-effector trajectories (positions, velocities, accelerations) to separate plots.
        """
        with self.lock:
            # Convert deques to lists
            timestamps = list(self.timestamps)
            if len(timestamps) == 0:
                rospy.logwarn("No data available to plot.")
                return

            # Joint data
            joint_positions_data = {j: list(self.joint_positions[j]) for j in JOINT_NAMES}
            joint_velocities_data = {j: list(self.joint_velocities[j]) for j in JOINT_NAMES}
            joint_accelerations_data = {j: list(self.joint_accelerations[j]) for j in JOINT_NAMES}

            # EE data
            ee_x = list(self.ee_x)
            ee_y = list(self.ee_y)
            ee_z = list(self.ee_z)
            ee_vx = list(self.ee_vx)
            ee_vy = list(self.ee_vy)
            ee_vz = list(self.ee_vz)
            ee_ax = list(self.ee_ax)
            ee_ay = list(self.ee_ay)
            ee_az = list(self.ee_az)

        ee_pos_data = {
            "ee_x": ee_x,
            "ee_y": ee_y,
            "ee_z": ee_z,
        }

        # Save EE position data to CSV
        self.save_ee_pos_to_csv(f"ee_positions_{self.naming_timestamp}.csv", timestamps, ee_pos_data)

        # Plot Joint Positions, Velocities, Accelerations (each joint in a separate subplot)
        self.plot_joint_data(
            timestamps, joint_positions_data, "Joint Positions Over Time", "Position (rad)", "joint_positions_plot"
        )
        self.plot_joint_data(
            timestamps, joint_velocities_data, "Joint Velocities Over Time", "Velocity (rad/s)", "joint_velocities_plot"
        )
        self.plot_joint_data(
            timestamps,
            joint_accelerations_data,
            "Joint Accelerations Over Time",
            "Acceleration (rad/s^2)",
            "joint_accelerations_plot",
        )

        # Plot End-Effector Positions, Velocities, Accelerations (x, y, z in separate subplots)
        self.plot_ee_data(
            timestamps, ee_x, ee_y, ee_z, "End-Effector Positions Over Time", "Position (m)", "ee_positions_plot"
        )
        self.plot_ee_data(
            timestamps, ee_vx, ee_vy, ee_vz, "End-Effector Velocities Over Time", "Velocity (m/s)", "ee_velocities_plot"
        )
        self.plot_ee_data(
            timestamps,
            ee_ax,
            ee_ay,
            ee_az,
            "End-Effector Accelerations Over Time",
            "Acceleration (m/s^2)",
            "ee_accelerations_plot",
        )

    def save_ee_pos_to_csv(self, filename, timestamps, ee_pos_data):
        """
        Saves end-effector position data (x, y, z) to a CSV file.
        """
        output_dir = "output_data"
        os.makedirs(output_dir, exist_ok=True)

        filepath = os.path.join(output_dir, filename)

        with open(filepath, mode="w", newline="") as file:
            writer = csv.writer(file)
            # Write header
            header = ["Timestamp"] + list(ee_pos_data.keys())
            writer.writerow(header)
            # Write rows
            for i in range(len(timestamps)):
                row = [timestamps[i]] + [ee_pos_data[key][i] for key in ee_pos_data]
                writer.writerow(row)

        rospy.loginfo(f"End-effector position data saved to {filepath}")


def main() -> None:
    """
    Main function to initialize the JointStatePlotter node, run it for a specified duration,
    and then save and plot the joint positions, velocities, accelerations and end-effector trajectories.
    """
    rospy.init_node("joint_state_plotter_node", anonymous=True)
    buffer_size = 10000
    plot_duration = 30

    plotter = JointStatePlotter(buffer_size=buffer_size)
    plotter.save_plots = True
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
