#!/usr/bin/env python

import rospy
import kdl_parser_py.urdf
from PyKDL import ChainFkSolverPos_recursive, ChainIkSolverPos_LMA, JntArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
import matplotlib.pyplot as plt
import numpy as np
import PyKDL as KDL

# Initialize the figure for live plotting
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_xlim(-0.8, 0.8)
ax.set_ylim(-0.8, 0.8)
ax.set_zlim(-0.2, 2)
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
scatter, = ax.plot([], [], [], 'ro')


# UR5 joint names
JOINT_NAMES = [
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
]

def pose_to_kdl_frame(pose):
    """Convert geometry_msgs/Pose to PyKDL Frame."""
    return KDL.Frame(
        KDL.Rotation.Quaternion(
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w
        ),
        KDL.Vector(pose.position.x, pose.position.y, pose.position.z)
    )

def interpolate_poses(start, end, steps):
    """Generate interpolated Cartesian poses."""
    waypoints = []
    for t in np.linspace(0, 1, steps):
        interp_pose = Pose()
        interp_pose.position.x = (1 - t) * start.position.x + t * end.position.x
        interp_pose.position.y = (1 - t) * start.position.y + t * end.position.y
        interp_pose.position.z = (1 - t) * start.position.z + t * end.position.z
        interp_pose.orientation = start.orientation  # Assumes constant orientation
        waypoints.append(interp_pose)
    return waypoints

def main():
    rospy.init_node('ur5_motion_node')
    
    # Load robot description
    robot_description = rospy.get_param("robot_description")
    ok, tree = kdl_parser_py.urdf.treeFromString(robot_description)
    if not ok:
        rospy.logerr("Failed to parse URDF")
        return

    # Extract KDL chain
    base_link = "base_link"
    end_effector_link = "wrist_3_link"
    kdl_chain = tree.getChain(base_link, end_effector_link)
    
    # Initialize solvers
    fk_solver = ChainFkSolverPos_recursive(kdl_chain)
    ik_solver = ChainIkSolverPos_LMA(kdl_chain)

    num_joints = kdl_chain.getNrOfJoints()
    initial_joints = JntArray(num_joints)
    joint_positions = KDL.JntArray(num_joints)
    

    # Subscriber to joint states
    def joint_states_callback(msg):
        """Update joint positions from /joint_states topic."""
        if len(msg.position) >= num_joints:
            for i in range(num_joints):
                joint_positions[i] = msg.position[i]

    rospy.Subscriber('/joint_states', JointState, joint_states_callback)
    
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

    start_frame = pose_to_kdl_frame(start_pose)
    end_frame = pose_to_kdl_frame(end_pose)

    # Solve IK for start and end
    start_joints = JntArray(num_joints)
    end_joints = JntArray(num_joints)

    if ik_solver.CartToJnt(initial_joints, start_frame, start_joints) < 0:
        rospy.logerr("Failed to compute IK for start pose")
        return

    if ik_solver.CartToJnt(start_joints, end_frame, end_joints) < 0:
        rospy.logerr("Failed to compute IK for end pose")
        return

    # Generate waypoints
    num_waypoints = 50
    waypoints = interpolate_poses(start_pose, end_pose, num_waypoints)
    
    # Solve IK for each waypoint
    joint_trajectory = []
    for waypoint in waypoints:
        frame = pose_to_kdl_frame(waypoint)
        waypoint_joints = JntArray(num_joints)
        if ik_solver.CartToJnt(initial_joints, frame, waypoint_joints) < 0:
            rospy.logerr("IK failed for a waypoint")
            return
        joint_trajectory.append([waypoint_joints[i] for i in range(num_joints)])
    
    # Publish trajectory
    trajectory_pub = rospy.Publisher('/eff_joint_traj_controller/command', JointTrajectory, queue_size=10)
    rospy.sleep(1)  # Wait for the publisher to initialize

    trajectory_msg = JointTrajectory()
    trajectory_msg.joint_names = JOINT_NAMES

    for i, joints in enumerate(joint_trajectory):
        point = JointTrajectoryPoint()
        point.positions = joints
        point.velocities = [0.1] * num_joints
        point.accelerations = [0.1] * num_joints
        point.time_from_start = rospy.Duration(i * 0.1)
        trajectory_msg.points.append(point)

    trajectory_pub.publish(trajectory_msg)
    rospy.loginfo("Trajectory published")

    rate = rospy.Rate(10)  # Log at 10 Hz
    x_positions = []
    y_positions = []
    z_positions = []
    while not rospy.is_shutdown():
        # Compute FK for the current joint positions
        end_effector_frame = KDL.Frame()
        if fk_solver.JntToCart(joint_positions, end_effector_frame) >= 0:
            position = end_effector_frame.p
            rotation = end_effector_frame.M
            quaternion = rotation.GetQuaternion()

            x_positions.append(position.x())
            y_positions.append(position.y())
            z_positions.append(position.z())
            # Update the 3D plot
            scatter.set_data(x_positions, y_positions)
            scatter.set_3d_properties(z_positions)
            plt.draw()
            plt.pause(0.01)  # Pause to allow the plot to update

            rospy.loginfo(
                f"End Effector Position: x={position.x() :.2f}, y={position.y() :.2f}, z={position.z() :.2f}\n"
                f"Orientation (quaternion): x={quaternion[0] :.2f}, y={quaternion[1] :.2f}, "
                f"z={quaternion[2] :.2f}, w={quaternion[3] :.2f}"
            )
        else:
            rospy.logwarn("Failed to compute FK for the current joint positions")
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
