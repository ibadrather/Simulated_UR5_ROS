#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose

def publish_pose():
    # Initialize the ROS node
    rospy.init_node('pose_publisher_node', anonymous=True)
    
    # Create a publisher to the /target_pose topic
    pose_pub = rospy.Publisher('/target_pose', Pose, queue_size=10)
    
    # Define the target pose
    target_pose = Pose()
    target_pose.position.x = 0.1
    target_pose.position.y = 0.1
    target_pose.position.z = 0.69
    target_pose.orientation.x = 0.0
    target_pose.orientation.y = 0.0
    target_pose.orientation.z = 0.0
    target_pose.orientation.w = 1.0  # Identity quaternion
    
    # Set the rate at which we want to publish the pose (1 Hz)
    rate = rospy.Rate(1)
    
    # While the ROS node is running, publish the pose
    while not rospy.is_shutdown():
        # Publish the pose message
        rospy.loginfo("Publishing Pose: Position [%.2f, %.2f, %.2f], Orientation [%.2f, %.2f, %.2f, %.2f]",
                      target_pose.position.x, target_pose.position.y, target_pose.position.z,
                      target_pose.orientation.x, target_pose.orientation.y, target_pose.orientation.z, target_pose.orientation.w)
        
        # Publish the pose to the /target_pose topic
        pose_pub.publish(target_pose)
        
        # Sleep to maintain the desired rate
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_pose()
    except rospy.ROSInterruptException:
        pass
