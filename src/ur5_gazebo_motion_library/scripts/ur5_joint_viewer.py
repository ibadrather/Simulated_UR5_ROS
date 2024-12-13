#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
from collections import deque
import matplotlib.pyplot as plt
import time


class JointStatePlotter:
    def __init__(self, buffer_size=100):
        self.buffer_size = buffer_size
        self.joint_buffers = {}  # Dictionary to store deque for each joint
        self.start_time = time.time()  # Record the start time
        self.fig, self.ax = plt.subplots()  # Set up matplotlib figure
        self.initialized = False

        # Subscribe to the joint_states topic
        rospy.Subscriber("/joint_states", JointState, self.joint_state_callback)

    def joint_state_callback(self, msg: JointState):
        """Callback to handle incoming joint states."""
        # Initialize joint buffers if not already done
        if not self.initialized:
            for joint_name in msg.name:
                self.joint_buffers[joint_name] = deque()
            self.initialized = True

        # Update the joint buffers with the latest positions
        for joint_name, position in zip(msg.name, msg.position):
            self.joint_buffers[joint_name].append(position)

    def save_and_plot(self):
        """Save the joint data and display the plot."""
        for joint_name, buffer in self.joint_buffers.items():
            # Plot the data for each joint
            self.ax.plot(list(buffer), label=joint_name)

        self.ax.legend()
        self.ax.set_title("Joint Positions Over Time")
        self.ax.set_xlabel("Time Steps")
        self.ax.set_ylabel("Position")
        plt.savefig("joint_positions_plot.png", dpi=600)  # Save the plot as an image
        plt.show()  # Display the plot


def main():
    rospy.init_node("joint_state_plotter_node", anonymous=True)
    jsp = JointStatePlotter()
    duration = 15  # Duration to run the node (in seconds)

    rate = rospy.Rate(50)  # 10 Hz loop rate
    start_time = time.time()

    try:
        while not rospy.is_shutdown() and (time.time() - start_time < duration):
            rate.sleep()
    except rospy.ROSInterruptException:
        pass

    # Save and display the plot after 30 seconds
    jsp.save_and_plot()


if __name__ == "__main__":
    main()
