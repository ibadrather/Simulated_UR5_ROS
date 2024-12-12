#!/usr/bin/env python3

import rospy
import kdl_parser_py.urdf
from PyKDL import ChainFkSolverPos_recursive, ChainIkSolverPos_LMA, JntArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
import matplotlib.pyplot as plt
import numpy as np
import PyKDL as KDL

# UR5 joint names
JOINT_NAMES = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]

class PoseUtils:

    def pose_to_kdl_frame(self, pose):
        """Convert geometry_msgs/Pose to PyKDL Frame."""
        return KDL.Frame(
            KDL.Rotation.Quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w),
            KDL.Vector(pose.position.x, pose.position.y, pose.position.z)
        )

    def interpolate_poses(self, start, end, steps):
        """Generate interpolated Cartesian poses linearly between start and end."""
        waypoints = []

        # Identify axes that are changing
        x_changing = not np.isclose(start.position.x, end.position.x)
        y_changing = not np.isclose(start.position.y, end.position.y)
        z_changing = not np.isclose(start.position.z, end.position.z)

        rospy.loginfo(f"Interpolating poses: X axis changing: {x_changing}, Y axis changing: {y_changing}, Z axis changing: {z_changing}")

        for t in np.linspace(0, 1, steps):
            interp_pose = Pose()
            interp_pose.position.x = (1 - t)*start.position.x + t*end.position.x if x_changing else start.position.x
            interp_pose.position.y = (1 - t)*start.position.y + t*end.position.y if y_changing else start.position.y
            interp_pose.position.z = (1 - t)*start.position.z + t*end.position.z if z_changing else start.position.z
            interp_pose.orientation = start.orientation
            waypoints.append(interp_pose)

        return waypoints

class InverseKinematics:
    def __init__(self, kdl_chain):
        self.ik_solver = ChainIkSolverPos_LMA(kdl_chain)
        self.fk_solver = ChainFkSolverPos_recursive(kdl_chain)
        self.num_joints = kdl_chain.getNrOfJoints()

    def solve_ik(self, initial_joints, frame):
        """Solve inverse kinematics for a given pose."""
        joints = JntArray(self.num_joints)
        if self.ik_solver.CartToJnt(initial_joints, frame, joints) < 0:
            raise ValueError("Failed to compute IK")
        return joints

    def solve_fk(self, joint_positions):
        """Solve forward kinematics for given joint positions."""
        frame = KDL.Frame()
        if self.fk_solver.JntToCart(joint_positions, frame) >= 0:
            return frame
        raise ValueError("Failed to compute FK")

class ROSNode:
    def __init__(self):
        rospy.init_node('ur5_motion_node')
        self.joint_positions = None

    def get_joint_positions(self, num_joints):
        """Subscribe to joint states and get current joint positions."""
        def callback(msg):
            if len(msg.position) >= num_joints:
                self.joint_positions = JntArray(num_joints)
                for i in range(num_joints):
                    self.joint_positions[i] = msg.position[i]

        rospy.Subscriber('/joint_states', JointState, callback)

    def publish_trajectory(self, trajectory):
        """Publish joint trajectory."""
        pub = rospy.Publisher('/eff_joint_traj_controller/command', JointTrajectory, queue_size=10)
        rospy.sleep(1)  # Wait for the publisher to initialize
        pub.publish(trajectory)

class Plotter:
    def __init__(self):
        self.fig, self.axes = plt.subplots(1, 3, figsize=(15, 5))
        self.ax_xy, self.ax_yz, self.ax_xz = self.axes

        # Configure XY plot
        self.ax_xy.set_xlim(-0.8, 0.8)
        self.ax_xy.set_ylim(-0.8, 0.8)
        self.ax_xy.set_xlabel('X')
        self.ax_xy.set_ylabel('Y')
        self.scatter_xy, = self.ax_xy.plot([], [], '-')

        # Configure YZ plot
        self.ax_yz.set_xlim(-0.8, 0.8)
        self.ax_yz.set_ylim(-0.2, 2)
        self.ax_yz.set_xlabel('Y')
        self.ax_yz.set_ylabel('Z')
        self.scatter_yz, = self.ax_yz.plot([], [], '-')

        # Configure XZ plot
        self.ax_xz.set_xlim(-0.8, 0.8)
        self.ax_xz.set_ylim(-0.2, 2)
        self.ax_xz.set_xlabel('X')
        self.ax_xz.set_ylabel('Z')
        self.scatter_xz, = self.ax_xz.plot([], [], '-')

        plt.ion()
        plt.show()

    def update_plot(self, x_positions, y_positions, z_positions):
        # Update XY scatter plot
        self.scatter_xy.set_data(x_positions, y_positions)
        # Removed relim() and autoscale_view()

        # Update YZ scatter plot
        self.scatter_yz.set_data(y_positions, z_positions)
        # Removed relim() and autoscale_view()

        # Update XZ scatter plot
        self.scatter_xz.set_data(x_positions, z_positions)
        # Removed relim() and autoscale_view()

        # Redraw the figure
        plt.draw()
        plt.pause(0.01)

if __name__ == '__main__':
    try:
        node = ROSNode()
        pose_util = PoseUtils()

        # Load robot description and initialize KDL chain
        robot_description = rospy.get_param("robot_description")
        ok, tree = kdl_parser_py.urdf.treeFromString(robot_description)
        if not ok:
            rospy.logerr("Failed to parse URDF")
            exit()

        kdl_chain = tree.getChain("base_link", "wrist_3_link")
        ik_solver = InverseKinematics(kdl_chain)
        node.get_joint_positions(kdl_chain.getNrOfJoints())

        # Define start and end poses
        start_pose = Pose()
        start_pose.position.x = 0.4
        start_pose.position.y = 0.6
        start_pose.position.z = 0.1
        start_pose.orientation.w = 1.0

        end_pose = Pose()
        end_pose.position.x = 0.4
        end_pose.position.y = -0.6
        end_pose.position.z = 0.1
        end_pose.orientation.w = 1.0

        start_frame = pose_util.pose_to_kdl_frame(start_pose)
        end_frame = pose_util.pose_to_kdl_frame(end_pose)

        # Solve IK for start and end
        start_joints = ik_solver.solve_ik(JntArray(ik_solver.num_joints), start_frame)
        # end_joints are solved but not necessarily needed directly, as we will go through waypoints
        end_joints = ik_solver.solve_ik(start_joints, end_frame)

        # Generate waypoints
        waypoints = pose_util.interpolate_poses(start_pose, end_pose, 200)

        # OPTIONAL: Plot ideal waypoints before executing
        x_wp = [w.position.x for w in waypoints]
        y_wp = [w.position.y for w in waypoints]
        z_wp = [w.position.z for w in waypoints]
        plt.figure()
        plt.subplot(1,3,1)
        plt.plot(x_wp, y_wp, 'o-')
        plt.title('XY Plane (Ideal)')
        plt.subplot(1,3,2)
        plt.plot(y_wp, z_wp, 'o-')
        plt.title('YZ Plane (Ideal)')
        plt.subplot(1,3,3)
        plt.plot(x_wp, z_wp, 'o-')
        plt.title('XZ Plane (Ideal)')
        plt.show()

        # Solve IK for each waypoint using previous solution as the seed
        joint_trajectory = []
        current_joints = start_joints
        for i, waypoint in enumerate(waypoints):
            rospy.loginfo(f"Waypoint {i}: {waypoint.position.x}, {waypoint.position.y}, {waypoint.position.z}")
            
            frame = pose_util.pose_to_kdl_frame(waypoint)

            # Use current_joints as the seed for IK
            try:
                current_joints = ik_solver.solve_ik(current_joints, frame)
                joint_positions = [current_joints[j] for j in range(ik_solver.num_joints)]
                rospy.loginfo(f"Joint positions for waypoint {i}: {joint_positions}")
                joint_trajectory.append(joint_positions)
            except ValueError as e:
                rospy.logerr(f"IK failed at waypoint {i}: {e}")

        # Publish trajectory
        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = JOINT_NAMES
        for i, joints in enumerate(joint_trajectory):
            point = JointTrajectoryPoint()
            point.positions = joints
            point.velocities = [0.1] * ik_solver.num_joints
            point.accelerations = [0.1] * ik_solver.num_joints
            point.time_from_start = rospy.Duration(i * 0.1)
            trajectory_msg.points.append(point)

        node.publish_trajectory(trajectory_msg)
        rospy.loginfo("Trajectory published")

        # Plotting live
        plotter = Plotter()
        x_positions, y_positions, z_positions = [], [], []
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            if node.joint_positions:
                frame = ik_solver.solve_fk(node.joint_positions)
                position = frame.p

                x_positions.append(position.x())
                y_positions.append(position.y())
                z_positions.append(position.z())
                plotter.update_plot(x_positions, y_positions, z_positions)

                rospy.loginfo(
                    f"End Effector Position: x={position.x() :.2f}, y={position.y() :.2f}, z={position.z() :.2f}\n"
                )

            rate.sleep()

    except rospy.ROSInterruptException:
        pass
