#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <cmath>
#include <vector>

// Constants for joint names and loop frequency
const std::vector<std::string> JOINT_NAMES = {
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint"
};
const double LOOP_RATE_HZ = 10.0; // 10 Hz

/**
 * Generate joint positions based on sine and cosine waves.
 * @return A vector of joint positions.
 */
std::vector<double> generateJointPositions()
{
    double time_now = ros::Time::now().toSec();
    return {
        std::sin(time_now),  // shoulder_pan_joint
        std::cos(time_now),  // shoulder_lift_joint
        std::sin(time_now),  // elbow_joint
        std::cos(time_now),  // wrist_1_joint
        std::sin(time_now),  // wrist_2_joint
        std::cos(time_now)   // wrist_3_joint
    };
}

int main(int argc, char** argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "sine_wave_control");
    ros::NodeHandle nh;

    // Create a publisher to the /eff_joint_traj_controller/command topic
    ros::Publisher joint_pub = nh.advertise<trajectory_msgs::JointTrajectory>(
        "/eff_joint_traj_controller/command", 10);

    // Set the loop rate
    ros::Rate loop_rate(LOOP_RATE_HZ);

    // Initialize the JointTrajectory message
    trajectory_msgs::JointTrajectory joint_trajectory;
    joint_trajectory.joint_names = JOINT_NAMES;

    // Initialize the JointTrajectoryPoint
    trajectory_msgs::JointTrajectoryPoint point;
    point.time_from_start = ros::Duration(1.0); // Specify time for this point

    while (ros::ok())
    {
        // Generate joint positions based on sine wave
        point.positions = generateJointPositions();

        // Clear and update trajectory points
        joint_trajectory.points.clear();
        joint_trajectory.points.push_back(point);

        // Publish the joint trajectory
        joint_pub.publish(joint_trajectory);

        // Sleep to maintain the loop rate
        loop_rate.sleep();
    }

    return 0;
}
