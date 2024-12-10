#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/jntarray.hpp>
#include <std_msgs/Empty.h>

const std::vector<std::string> JOINT_NAMES = {
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
};

// Function to compute inverse kinematics (IK)
bool computeIK(const KDL::Chain& chain, const KDL::JntArray& initial_joints,
               const geometry_msgs::Pose& target_pose, KDL::JntArray& result_joints) {
    KDL::ChainIkSolverPos_LMA ik_solver(chain);

    // Convert target_pose to KDL::Frame
    KDL::Frame target_frame(
        KDL::Rotation::Quaternion(
            target_pose.orientation.x,
            target_pose.orientation.y,
            target_pose.orientation.z,
            target_pose.orientation.w
        ),
        KDL::Vector(
            target_pose.position.x,
            target_pose.position.y,
            target_pose.position.z
        )
    );

    // Solve IK
    if (ik_solver.CartToJnt(initial_joints, target_frame, result_joints) < 0) {
        ROS_ERROR("Inverse kinematics computation failed for target position.");
        return false;
    }
    ROS_INFO("IK computation successful.");
    return true;
}

// Function to publish joint trajectory to move robot
void publishTrajectory(const ros::Publisher& joint_pub,
                       const KDL::JntArray& joint_positions) {
    trajectory_msgs::JointTrajectory trajectory_msg;
    trajectory_msg.joint_names = JOINT_NAMES;

    trajectory_msgs::JointTrajectoryPoint point;

    // Populate joint positions
    for (unsigned int i = 0; i < joint_positions.rows(); ++i) {
        point.positions.push_back(joint_positions(i));
    }

    point.velocities.resize(JOINT_NAMES.size(), 0.0);
    point.accelerations.resize(JOINT_NAMES.size(), 0.0);
    point.time_from_start = ros::Duration(2.0); // Adjust duration as needed

    trajectory_msg.points.push_back(point);
    joint_pub.publish(trajectory_msg);
    ROS_INFO("Published trajectory to target position.");
}

// Function to continuously publish current position and joint states
void publishCurrentPosition(const ros::Publisher& joint_pub, const KDL::Chain& chain,
                            const KDL::JntArray& joint_positions) {
    KDL::ChainFkSolverPos_recursive fk_solver(chain);
    KDL::Frame current_pose;
    fk_solver.JntToCart(joint_positions, current_pose);

    // Convert current pose to geometry_msgs::Pose
    geometry_msgs::Pose current_pose_msg;
    current_pose_msg.position.x = current_pose.p.x();
    current_pose_msg.position.y = current_pose.p.y();
    current_pose_msg.position.z = current_pose.p.z();
    
    // Get quaternion from rotation matrix
    double x, y, z, w;
    current_pose.M.GetQuaternion(x, y, z, w);
    current_pose_msg.orientation.x = x;
    current_pose_msg.orientation.y = y;
    current_pose_msg.orientation.z = z;
    current_pose_msg.orientation.w = w;

    ROS_INFO("Current position: [%.2f, %.2f, %.2f]", current_pose_msg.position.x, current_pose_msg.position.y, current_pose_msg.position.z);

    // Publish joint states
    publishTrajectory(joint_pub, joint_positions);
}

// Callback function to move to a target pose
void moveToPoseCallback(const geometry_msgs::Pose::ConstPtr& target_pose_msg,
                        const ros::Publisher& joint_pub, const KDL::Chain& chain, KDL::JntArray& initial_joints) {
    KDL::JntArray result_joints(chain.getNrOfJoints());

    ROS_INFO("Requested to go to a pose");

    if (computeIK(chain, initial_joints, *target_pose_msg, result_joints)) {
        publishTrajectory(joint_pub, result_joints);
    } else {
        ROS_ERROR("Failed to compute IK for target pose.");
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "cartesian_position_node");
    ros::NodeHandle nh;

    // Load robot description and create KDL chain
    std::string robot_description;
    nh.getParam("robot_description", robot_description);

    KDL::Tree kdl_tree;
    if (!kdl_parser::treeFromString(robot_description, kdl_tree)) {
        ROS_ERROR("Failed to parse URDF.");
        return -1;
    }

    KDL::Chain kdl_chain;
    if (!kdl_tree.getChain("base_link", "wrist_3_link", kdl_chain)) {
        ROS_ERROR("Failed to extract KDL chain.");
        return -1;
    }

    ros::Publisher joint_pub = nh.advertise<trajectory_msgs::JointTrajectory>(
        "/eff_joint_traj_controller/command", 10);

    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::Pose>(
        "/target_pose", 10, boost::bind(moveToPoseCallback, _1, joint_pub, kdl_chain, KDL::JntArray(kdl_chain.getNrOfJoints())));

    // Define start pose
    geometry_msgs::Pose start_pose;
    start_pose.position.x = 0.5;
    start_pose.position.y = 0.5;
    start_pose.position.z = 0.5;
    start_pose.orientation.w = 1.0;

    // Initial joint positions
    KDL::JntArray initial_joints(kdl_chain.getNrOfJoints());
    initial_joints.data.setZero();

    // Publish current position continuously
    ros::Rate loop_rate(10);  // 10 Hz
    while (ros::ok()) {
        // publishCurrentPosition(joint_pub, kdl_chain, initial_joints);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
