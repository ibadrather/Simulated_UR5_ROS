#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <math.h>

int main(int argc, char** argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "sine_wave_control");
    ros::NodeHandle nh;

    // Create a publisher to the /eff_joint_traj_controller/command topic
    ros::Publisher joint_pub = nh.advertise<trajectory_msgs::JointTrajectory>("/eff_joint_traj_controller/command", 10);

    // Set the loop rate
    ros::Rate loop_rate(10); // 10 Hz

    // Create the JointTrajectory message to publish
    trajectory_msgs::JointTrajectory joint_trajectory;
    joint_trajectory.joint_names = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", 
                                    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};

    trajectory_msgs::JointTrajectoryPoint point;

    while (ros::ok())
    {
        // Calculate sine wave joint positions
        point.positions.clear();
        point.positions.push_back(sin(ros::Time::now().toSec())); // shoulder_pan_joint
        point.positions.push_back(cos(ros::Time::now().toSec())); // shoulder_lift_joint
        point.positions.push_back(sin(ros::Time::now().toSec())); // elbow_joint
        point.positions.push_back(cos(ros::Time::now().toSec())); // wrist_1_joint
        point.positions.push_back(sin(ros::Time::now().toSec())); // wrist_2_joint
        point.positions.push_back(cos(ros::Time::now().toSec())); // wrist_3_joint

        point.time_from_start = ros::Duration(1.0); // Specify time for this point

        // Add the point to the trajectory
        joint_trajectory.points.clear();
        joint_trajectory.points.push_back(point);

        // Publish the joint trajectory
        joint_pub.publish(joint_trajectory);

        // Sleep to maintain the loop rate
        loop_rate.sleep();
    }

    return 0;
}
