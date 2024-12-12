#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose

class TargetPosePublisher:
    def __init__(self):
        rospy.init_node('target_pose_publisher', anonymous=True)
        self.pub = rospy.Publisher('/ur5_target_pose', Pose, queue_size=10)
        self.sub = rospy.Subscriber('/ur5_current_pose', Pose, self.current_pose_callback)
        self.rate = rospy.Rate(0.1)  # publish once every 10 seconds

        # Define a list of target poses to cycle through
        self.poses = []

        # Pose 1
        p1 = Pose()
        p1.position.x = 0.4
        p1.position.y = -0.6
        p1.position.z = 0.1
        p1.orientation.w = 1.0
        self.poses.append(p1)

        # Pose 2
        p2 = Pose()
        p2.position.x = 0.4
        p2.position.y = 0.0
        p2.position.z = 0.1
        p2.orientation.w = 1.0
        self.poses.append(p2)

        # Pose 3
        p3 = Pose()
        p3.position.x = 0.4
        p3.position.y = 0.6
        p3.position.z = 0.1
        p3.orientation.w = 1.0
        self.poses.append(p3)

        self.idx = 0
        rospy.loginfo("Starting to publish target poses and print current pose...")

    def current_pose_callback(self, pose):
        # Print current robot pose to the console
        rospy.loginfo(f"Current Robot Pose: "
                      f"x={pose.position.x:.3f}, "
                      f"y={pose.position.y:.3f}, "
                      f"z={pose.position.z:.3f}, "
                      f"qw={pose.orientation.w:.3f}, "
                      f"qx={pose.orientation.x:.3f}, "
                      f"qy={pose.orientation.y:.3f}, "
                      f"qz={pose.orientation.z:.3f}")

    def run(self):
        while not rospy.is_shutdown():
            # Publish next target pose
            target_pose = self.poses[self.idx]
            self.pub.publish(target_pose)
            rospy.loginfo(f"Published target pose {self.idx+1}: "
                          f"x={target_pose.position.x}, "
                          f"y={target_pose.position.y}, "
                          f"z={target_pose.position.z}")
            self.idx = (self.idx + 1) % len(self.poses)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        node = TargetPosePublisher()
        node.run()
    except rospy.ROSInterruptException:
        pass
