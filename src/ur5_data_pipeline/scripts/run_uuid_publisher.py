#!/usr/bin/env python
"""
ROS node to generate and publish a unique UUID for a session run.

This node publishes a unique UUID to the `/run_uuid` topic, ensuring it is available
before the start of other nodes. The UUID remains the same for the session until the node
is restarted to generate a new UUID for the next session.

Functions:
    uuid_publisher(): Initializes the ROS node, generates a UUID, and publishes it periodically.

Usage:
    To ensure proper node startup order, run this node first and then other nodes dependent
    on the `/run_uuid` topic. For example:

    1. Start this node:
        $ rosrun ur5_data_pipeline run_uuid_publisher.py

    2. Start subsequent nodes:
        $ rosrun ur5_data_pipeline image_data_saver.py
"""

import rospy
from std_msgs.msg import String
import uuid


def uuid_publisher():
    """
    Initializes the ROS node, generates a unique UUID, and publishes it to the `/run_uuid` topic.

    The UUID is generated once at the start of the node and remains consistent throughout the
    session. It is published periodically at 0.2 Hz to ensure availability for other nodes.

    Raises:
        rospy.ROSInterruptException: If the ROS node is interrupted during execution.
    """
    rospy.init_node("uuid_publisher", anonymous=False)
    pub = rospy.Publisher("/run_uuid", String, queue_size=1)

    # Generate the UUID
    run_uuid = str(uuid.uuid4())
    rospy.loginfo(f"Generated UUID: {run_uuid}")

    # Publish the UUID and keep publishing periodically
    rate = rospy.Rate(0.2)
    while not rospy.is_shutdown():
        pub.publish(run_uuid)
        rate.sleep()


if __name__ == "__main__":
    try:
        uuid_publisher()
    except rospy.ROSInterruptException:
        pass
