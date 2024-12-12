#!/usr/bin/env python3


#############################################################################
#############################################################################
#############################################################################


# import rospy
# from sqlalchemy import create_engine, Column, Integer, String, Float
# from sqlalchemy.orm import sessionmaker, declarative_base
# from influxdb_client_3 import InfluxDBClient3, Point

# # Configuration for InfluxDB
# ORG = "Dev"
# BUCKET_NAME = "ur5_sim_data"
# HOST = "https://eu-central-1-1.aws.cloud2.influxdata.com"
# TOKEN = "YOUR_INFLUXDB_TOKEN"

# # Initialize InfluxDB client
# client = InfluxDBClient3(host=HOST, token=TOKEN, org=ORG)

# # Define base for SQLAlchemy models
# Base = declarative_base()

# class JointState(Base):
#     """
#     SQLAlchemy model to represent the joint state data in SQLite.
#     """
#     __tablename__ = 'joint_states'

#     id = Column(Integer, primary_key=True, autoincrement=True)
#     joint_name = Column(String)
#     position = Column(Float)
#     velocity = Column(Float)
#     effort = Column(Float)
#     timestamp = Column(Float)

# class DatabaseSaver:
#     def __init__(self, sqlite_db_path: str, run_uuid:str, use_influxdb=False):
#         self.use_influxdb = use_influxdb
#         self.run_uuid = run_uuid
#         if self.use_influxdb:
#             self.client = InfluxDBClient3(
#                 host=HOST, token=TOKEN, org=ORG
#             )
#             self.bucket_name = BUCKET_NAME
#         else:
#             # Set up SQLite using SQLAlchemy
#             self.engine = create_engine(f"sqlite:///{sqlite_db_path}", echo=True)
#             Base.metadata.create_all(self.engine)
#             Session = sessionmaker(bind=self.engine)
#             self.session = Session()

#     def save_joint_states(self, joint_states):
#         """
#         Save joint state data either to InfluxDB or SQLite depending on configuration.

#         Args:
#             joint_states (JointState): The message from the `/joint_states` topic.
#         """
#         timestamp = rospy.Time.now().to_sec()
#         try:
#             for i, joint_name in enumerate(joint_states.name):
#                 if self.use_influxdb:
#                     # Prepare and write to InfluxDB
#                     point = (
#                         Point("joint_states")
#                         .tag("joint_name", joint_name)
#                         .field("position", joint_states.position[i])
#                         .field("velocity", joint_states.velocity[i])
#                         .field("effort", joint_states.effort[i])
#                         .field("run_uuid", self.run_uuid)
#                         .time(timestamp, write_precision="ms")
#                     )
#                     # Write the point to InfluxDB
#                     self.client.write(self.bucket_name, record=point)
#                 else:
#                     # Prepare and insert into SQLite
#                     joint_state = JointState(
#                         joint_name=joint_name,
#                         position=joint_states.position[i],
#                         velocity=joint_states.velocity[i],
#                         effort=joint_states.effort[i],
#                         timestamp=timestamp
#                     )
#                     self.session.add(joint_state)
#             self.session.commit()
#             rospy.loginfo("Successfully saved joint states.")
#         except Exception as e:
#             rospy.logerr(f"Failed to save joint states: {e}")
#             self.session.rollback()

#     def close(self):
#         """
#         Close the database connection (if SQLite).
#         """
#         if not self.use_influxdb:
#             self.session.close()

#############################################################################
#############################################################################
#############################################################################


import rospy
from sensor_msgs.msg import JointState
import yaml

from std_msgs.msg import String
from pathlib import Path

import os
import sys

sys.path.append(os.path.dirname(os.path.abspath(__file__)))

print("#" * 50)
print(os.path.dirname(os.path.abspath(__file__)))

from ur5_data_pipeline.scripts.DatabaseSaver import DatabaseSaver


def load_influxdb_config():
    """
    Load the InfluxDB configuration from the ROS parameter server.
    """
    try:
        config_path = rospy.get_param("~influxdb_config_path")
        rospy.loginfo(f"Loading InfluxDB config from: {config_path}")

        with open(config_path, "r") as file:
            config = yaml.safe_load(file)

        influx_config = config.get("influxdb")
        if not influx_config:
            rospy.logerr("InfluxDB configuration not found in the YAML file.")
            raise ValueError("InfluxDB configuration missing.")

        return influx_config
    except Exception as e:
        rospy.logerr(f"Failed to load InfluxDB config: {e}")
        raise


class UR5JointStateSaver:
    def __init__(self, use_influxdb: bool, base_dir: str = "~/runs"):

        self.run_id = None  # Placeholder for the UUID
        self.base_dir = Path(base_dir).expanduser()

        # Subscribe to the UUID topic
        rospy.Subscriber("/run_uuid", String, self.uuid_callback)

        # Wait until UUID is received
        rospy.loginfo("Waiting for UUID...")
        while self.run_id is None and not rospy.is_shutdown():
            rospy.sleep(0.1)
        rospy.loginfo(f"Received UUID: {self.run_id}")

        # Determine save path for SQLite DB
        self.database_save_path = self.base_dir / self.run_id / "joint_state_data.db"

        rospy.loginfo(f"Received UUID: {self.run_id}")

        if use_influxdb:
            rospy.loginfo("Saving UR5 Joint State Data to InfluxDB...")
        else:
            rospy.loginfo(f"Saving UR5 Joint State Data to SQLite: {self.database_save_path}")

        # Initialize DatabaseSaver
        self.database_saver = DatabaseSaver(
            use_influxdb=use_influxdb, run_uuid=self.run_id, sqlite_db_path=str(self.database_save_path)
        )

        # Subscribe to joint states topic
        rospy.Subscriber("/joint_states", JointState, self.joint_state_callback)

        # Register shutdown callback
        rospy.on_shutdown(self.shutdown)

    def uuid_callback(self, msg: String):
        """
        Callback to receive the UUID from the `/run_uuid` topic.

        Args:
            msg (String): The UUID message.
        """
        self.run_id = msg.data
        rospy.loginfo(f"Received UUID: {self.run_id}")

    def joint_state_callback(self, msg: JointState):
        """
        Callback function for the `/joint_states` topic.

        Args:
            msg (JointState): The message from the `/joint_states` topic.
        """
        rospy.loginfo("Received Joint State Data")
        try:
            self.database_saver.save_joint_states(msg)
        except Exception as e:
            rospy.logerr(f"Error saving joint states: {e}")

    def shutdown(self):
        self.database_saver.close()


if __name__ == "__main__":
    try:
        use_influxdb = False
        run_data_saver_path = Path("/home/edreate/Desktop/Simulated_UR5_ROS/runs")

        rospy.init_node("joint_state_saver")
        ur5_saver = UR5JointStateSaver(use_influxdb, base_dir=run_data_saver_path)
        rospy.loginfo(f"Starting joint state listener with InfluxDB: {use_influxdb}")
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.logerr("ROS Interrupted Exception.")
    except Exception as e:
        rospy.logerr(f"An unexpected error occurred: {e}")
