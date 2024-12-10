#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/jntarray.hpp>

const std::vector<std::string> JOINT_NAMES = {
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
};

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

    // Define start and end poses
    geometry_msgs::Pose start_pose, end_pose;
    start_pose.position.x = 0.5;
    start_pose.position.y = 0.5;
    start_pose.position.z = 0.5;
    start_pose.orientation.w = 1.0;

    end_pose.position.x = -0.5;
    end_pose.position.y = -0.5;
    end_pose.position.z = 0.5;
    end_pose.orientation.w = 1.0;

    ROS_INFO("Start pose: [%.2f, %.2f, %.2f]", start_pose.position.x, start_pose.position.y, start_pose.position.z);
    ROS_INFO("End pose: [%.2f, %.2f, %.2f]", end_pose.position.x, end_pose.position.y, end_pose.position.z);

    KDL::JntArray initial_joints(kdl_chain.getNrOfJoints());
    initial_joints.data.setZero();

    geometry_msgs::Pose target_pose = start_pose;

    // Repeat back-and-forth motion
    int cycles = 10; // Number of cycles to repeat
    for (int i = 0; i < cycles && ros::ok(); ++i) {
        KDL::JntArray result_joints(kdl_chain.getNrOfJoints());
        if (computeIK(kdl_chain, initial_joints, target_pose, result_joints)) {
            publishTrajectory(joint_pub, result_joints);
            ROS_INFO("Moving to %s pose.", (target_pose.position.x == start_pose.position.x) ? "start" : "end");
        } else {
            ROS_ERROR("Failed to compute IK for target pose. Skipping...");
        }

        target_pose = (target_pose.position.x == start_pose.position.x) ? end_pose : start_pose;
        ros::Duration(10.0).sleep(); // Pause for motion execution
    }

    return 0;
}