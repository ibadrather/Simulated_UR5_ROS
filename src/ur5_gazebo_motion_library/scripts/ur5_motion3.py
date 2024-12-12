#!/usr/bin/env python3

import rospy
import kdl_parser_py.urdf
from PyKDL import ChainFkSolverPos_recursive, ChainIkSolverPos_LMA, JntArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
import numpy as np
import PyKDL as KDL

# UR5 joint names
JOINT_NAMES = ["shoulder_pan_joint",
               "shoulder_lift_joint",
               "elbow_joint",
               "wrist_1_joint",
               "wrist_2_joint",
               "wrist_3_joint"]

class PoseUtils:
    @staticmethod
    def pose_to_kdl_frame(pose):
        """Convert geometry_msgs/Pose to PyKDL Frame."""
        return KDL.Frame(
            KDL.Rotation.Quaternion(pose.orientation.x,
                                    pose.orientation.y,
                                    pose.orientation.z,
                                    pose.orientation.w),
            KDL.Vector(pose.position.x, pose.position.y, pose.position.z)
        )

    @staticmethod
    def interpolate_poses(start, end, steps=50):
        """Generate interpolated Cartesian poses linearly between start and end."""
        waypoints = []
        for t in np.linspace(0, 1, steps):
            interp_pose = Pose()
            interp_pose.position.x = (1 - t)*start.position.x + t*end.position.x
            interp_pose.position.y = (1 - t)*start.position.y + t*end.position.y
            interp_pose.position.z = (1 - t)*start.position.z + t*end.position.z
            # For orientation, just keep start orientation or slerp if needed
            # Here we keep orientation constant for simplicity
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
        out_joints = JntArray(self.num_joints)
        if self.ik_solver.CartToJnt(initial_joints, frame, out_joints) < 0:
            raise ValueError("Failed to compute IK")
        return out_joints

    def solve_fk(self, joint_positions):
        """Solve forward kinematics for given joint positions."""
        frame = KDL.Frame()
        if self.fk_solver.JntToCart(joint_positions, frame) >= 0:
            return frame
        raise ValueError("Failed to compute FK")

class UR5MotionNode:
    def __init__(self):
        rospy.init_node('ur5_motion_node', anonymous=True)

        # Load robot description and initialize KDL chain
        robot_description = rospy.get_param("robot_description", "")
        if not robot_description:
            rospy.logerr("robot_description not found on parameter server")
            raise rospy.ROSException("No robot_description found")

        ok, tree = kdl_parser_py.urdf.treeFromString(robot_description)
        if not ok:
            rospy.logerr("Failed to parse URDF")
            raise rospy.ROSException("Failed to parse URDF")

        self.kdl_chain = tree.getChain("base_link", "wrist_3_link")
        self.ik_solver = InverseKinematics(self.kdl_chain)
        self.pose_util = PoseUtils()

        self.num_joints = self.kdl_chain.getNrOfJoints()
        self.joint_positions = JntArray(self.num_joints)

        # Initial pose
        # For simplicity, assume a known initial joint configuration or wait until we get it from joint states
        self.current_pose = Pose()
        self.current_pose.position.x = 0.4
        self.current_pose.position.y = -0.6
        self.current_pose.position.z = 0.1
        self.current_pose.orientation.w = 1.0
        self.initialized = False

        # Publishers and Subscribers
        self.target_pose_sub = rospy.Subscriber('/ur5_target_pose', Pose, self.target_pose_callback)
        self.joint_state_sub = rospy.Subscriber('/joint_states', JointState, self.joint_state_callback)
        self.current_pose_pub = rospy.Publisher('/ur5_current_pose', Pose, queue_size=10)
        self.trajectory_pub = rospy.Publisher('/eff_joint_traj_controller/command', JointTrajectory, queue_size=10)

        # We keep a target pose once received
        self.new_target_pose = None

        # Setup a timer to publish current pose at a certain rate
        rospy.Timer(rospy.Duration(0.1), self.publish_current_pose)

        # Start from a known initial position:
        # Let's say the initial position matches the start_joints (all zeros)
        # In a real scenario, you'd want to retrieve the initial joint states and compute the current pose.
        # We'll just wait until we have joint states:
        rospy.loginfo("Waiting for initial joint states...")
        while not rospy.is_shutdown() and not self.initialized:
            rospy.sleep(0.1)
        rospy.loginfo("Initial joint states received, current pose established.")

    def joint_state_callback(self, msg):
        # Update current joint positions
        # Assume the order in msg matches JOINT_NAMES or a known order
        name_to_index = {n: i for i, n in enumerate(JOINT_NAMES)}
        if all(n in msg.name for n in JOINT_NAMES):
            for i, joint_name in enumerate(JOINT_NAMES):
                idx = msg.name.index(joint_name)
                self.joint_positions[i] = msg.position[idx]

            # Compute current pose from joint positions
            frame = self.ik_solver.solve_fk(self.joint_positions)
            self.current_pose.position.x = frame.p.x()
            self.current_pose.position.y = frame.p.y()
            self.current_pose.position.z = frame.p.z()
            qx, qy, qz, qw = frame.M.GetQuaternion()
            self.current_pose.orientation.x = qx
            self.current_pose.orientation.y = qy
            self.current_pose.orientation.z = qz
            self.current_pose.orientation.w = qw

            if not self.initialized:
                self.initialized = True

    def target_pose_callback(self, pose):
        rospy.loginfo(f"Received new target pose: {pose}")
        self.new_target_pose = pose
        self.execute_motion_to_target()

    def publish_current_pose(self, event):
        # Continuously publish current pose
        if self.initialized:
            self.current_pose_pub.publish(self.current_pose)

    def execute_motion_to_target(self):
        if self.new_target_pose is None:
            return

        # Interpolate poses from current pose to new_target_pose
        waypoints = self.pose_util.interpolate_poses(self.current_pose, self.new_target_pose, steps=50)

        # Solve IK for each waypoint
        # Use current joint positions as the seed
        current_joints = JntArray(self.num_joints)
        for i in range(self.num_joints):
            current_joints[i] = self.joint_positions[i]

        joint_trajectory = []
        for i, wp in enumerate(waypoints):
            frame = self.pose_util.pose_to_kdl_frame(wp)
            try:
                current_joints = self.ik_solver.solve_ik(current_joints, frame)
                joint_positions = [current_joints[j] for j in range(self.num_joints)]
                joint_trajectory.append(joint_positions)
            except ValueError as e:
                rospy.logerr(f"IK failed at waypoint {i}: {e}")
                return

        # Publish joint trajectory
        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = JOINT_NAMES
        # Create a simple time parameterization
        for i, joints in enumerate(joint_trajectory):
            point = JointTrajectoryPoint()
            point.positions = joints
            point.time_from_start = rospy.Duration(i * 0.05)  # 50 ms per step
            trajectory_msg.points.append(point)

        # Publish trajectory
        rospy.loginfo("Publishing trajectory to move to target pose...")
        # Give time for publishers to connect
        rospy.sleep(0.5)
        self.trajectory_pub.publish(trajectory_msg)

        # Wait for the motion to (approximately) complete
        # In a real scenario, you'd use feedback or action clients
        # Here, we just sleep for the duration of the trajectory + a buffer
        rospy.sleep(len(joint_trajectory)*0.05 + 1.0)

        # Once done, the current_pose should now be at the target_pose
        # Set current_pose = new_target_pose to avoid minor numerical discrepancies
        self.current_pose = self.new_target_pose
        self.new_target_pose = None
        rospy.loginfo("Reached target pose, waiting for next command.")

if __name__ == '__main__':
    try:
        node = UR5MotionNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
