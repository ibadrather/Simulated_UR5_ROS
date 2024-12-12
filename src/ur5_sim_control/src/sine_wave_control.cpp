#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <cmath>
#include <vector>

// Constants for joint names
const std::vector<std::string> JOINT_NAMES = {
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint"
};
const double LOOP_RATE_HZ = 50.0; // Higher frequency for smoother control

// Parameters for sine wave generation
struct SineWaveParams {
    double amplitude;
    double frequency;
    double phase;
};

// Initialize sine wave parameters for each joint
std::vector<SineWaveParams> sine_params = {
    {1.0, 1.0, 0.0},  // shoulder_pan_joint
    {1.0, 1.0, M_PI / 2}, // shoulder_lift_joint
    {1.0, 1.0, M_PI},     // elbow_joint
    {1.0, 1.0, M_PI / 4}, // wrist_1_joint
    {1.0, 1.0, 3 * M_PI / 4}, // wrist_2_joint
    {1.0, 1.0, M_PI / 2}  // wrist_3_joint
};

/**
 * Generate joint positions based on sine waves.
 * @param time_now The current time in seconds.
 * @return A vector of joint positions.
 */
std::vector<double> generateJointPositions(double time_now)
{
    std::vector<double> positions;
    for (const auto& param : sine_params) {
        positions.push_back(param.amplitude * std::sin(2 * M_PI * param.frequency * time_now + param.phase));
    }
    return positions;
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
        // Get the current time
        double time_now = ros::Time::now().toSec();

        // Generate joint positions based on sine wave
        point.positions = generateJointPositions(time_now);

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
