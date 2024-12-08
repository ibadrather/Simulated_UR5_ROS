#!/usr/bin/env python3






#############################################################################
#############################################################################
#############################################################################



import rospy
from sqlalchemy import create_engine, Column, Integer, String, Float
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.orm import sessionmaker
from influxdb_client_3 import InfluxDBClient3, Point

# Configuration for InfluxDB
ORG = "Dev"
BUCKET_NAME = "ur5_sim_data"
HOST = "https://eu-central-1-1.aws.cloud2.influxdata.com"
TOKEN = "YOUR_INFLUXDB_TOKEN"

# Initialize InfluxDB client
client = InfluxDBClient3(host=HOST, token=TOKEN, org=ORG)

# Define base for SQLAlchemy models
Base = declarative_base()

class JointState(Base):
    """
    SQLAlchemy model to represent the joint state data in SQLite.
    """
    __tablename__ = 'joint_states'
    
    id = Column(Integer, primary_key=True, autoincrement=True)
    joint_name = Column(String)
    position = Column(Float)
    velocity = Column(Float)
    effort = Column(Float)
    timestamp = Column(Float)

class DatabaseSaver:
    def __init__(self, sqlite_db_path: str, run_uuid:str, use_influxdb=False):
        self.use_influxdb = use_influxdb
        self.run_uuid = run_uuid
        if self.use_influxdb:
            self.client = InfluxDBClient3(
                host=HOST, token=TOKEN, org=ORG
            )
            self.bucket_name = BUCKET_NAME
        else:
            # Set up SQLite using SQLAlchemy
            self.engine = create_engine(f"sqlite:///{sqlite_db_path}/ur5_sim_data.db", echo=True)
            Base.metadata.create_all(self.engine)
            Session = sessionmaker(bind=self.engine)
            self.session = Session()

    def save_joint_states(self, joint_states):
        """
        Save joint state data either to InfluxDB or SQLite depending on configuration.

        Args:
            joint_states (JointState): The message from the `/joint_states` topic.
        """
        timestamp = rospy.Time.now().to_sec()
        try:
            for i, joint_name in enumerate(joint_states.name):
                if self.use_influxdb:
                    # Prepare and write to InfluxDB
                    point = (
                        Point("joint_states")
                        .tag("joint_name", joint_name)
                        .field("position", joint_states.position[i])
                        .field("velocity", joint_states.velocity[i])
                        .field("effort", joint_states.effort[i])
                        .field("run_uuid", self.run_uuid)
                        .time(timestamp, write_precision="ms")
                    )
                    # Write the point to InfluxDB
                    self.client.write(self.bucket_name, record=point)
                else:
                    # Prepare and insert into SQLite
                    joint_state = JointState(
                        joint_name=joint_name,
                        position=joint_states.position[i],
                        velocity=joint_states.velocity[i],
                        effort=joint_states.effort[i],
                        timestamp=timestamp
                    )
                    self.session.add(joint_state)
            self.session.commit()
            rospy.loginfo("Successfully saved joint states.")
        except Exception as e:
            rospy.logerr(f"Failed to save joint states: {e}")
            self.session.rollback()

    def close(self):
        """
        Close the database connection (if SQLite).
        """
        if not self.use_influxdb:
            self.session.close()

#############################################################################
#############################################################################
#############################################################################


import rospy
from sensor_msgs.msg import JointState
import yaml


from std_msgs.msg import String
from pathlib import Path
import logging
import os
import sys
#
# Set up logging configuration
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

def load_influxdb_config():
    """
    Load the InfluxDB configuration from the ROS parameter server.
    """
    try:
        config_path = rospy.get_param("~config_path")
        logger.info(f"Loading InfluxDB config from: {config_path}")
        
        with open(config_path, 'r') as file:
            config = yaml.safe_load(file)
        
        influx_config = config.get("influxdb")
        if not influx_config:
            logger.error("InfluxDB configuration not found in the YAML file.")
            raise ValueError("InfluxDB configuration missing.")
        
        return influx_config
    except Exception as e:
        logger.error(f"Failed to load InfluxDB config: {e}")
        raise


class UR5JointStateSaver:
    def __init__(self, use_influxdb: bool, base_dir: str = "~/runs"):
        self.run_id = None  # Placeholder for the UUID

        # Subscribe to the UUID topic
        rospy.Subscriber("/run_uuid", String, self.uuid_callback)
        
        # Wait until UUID is received
        rospy.loginfo("Waiting for UUID...")
        while self.run_id is None and not rospy.is_shutdown():
            rospy.sleep(0.1)
        
        # Determine save path for SQLite DB
        self.base_dir = Path(base_dir).expanduser()
        self.database_save_path = self.base_dir / self.run_id / "joint_state_data.db"

        # Initialize DatabaseSaver
        self.database_saver = DatabaseSaver(use_influxdb=use_influxdb, run_uuid=self.run_id, sqlite_db_path=str(self.database_save_path))

        if use_influxdb:
            logger.info("Saving UR5 Joint State Data to InfluxDB...")
        else:
            logger.info(f"Saving UR5 Joint State Data to SQLite: {self.database_save_path}")

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
            logger.error(f"Error saving joint states: {e}")

    def shutdown(self):
        self.database_saver.close()

def joint_state_listener(use_influxdb: bool):
    """
    Initializes the ROS node and subscribes to the `/joint_states` topic.
    
    Args:
        use_influxdb (bool): Whether to use InfluxDB or SQLite for saving data.
    """
    rospy.init_node("joint_state_saver", anonymous=False)

    # Initialize the UR5JointStateSaver class with the selected database type
    ur5_saver = UR5JointStateSaver(use_influxdb)
    rospy.spin()

if __name__ == "__main__":
    try:
        use_influxdb = rospy.get_param("~use_influxdb", False)  # Default to InfluxDB if not specified
        logger.info(f"Starting joint state listener with InfluxDB: {use_influxdb}")
        joint_state_listener(use_influxdb)
    except rospy.ROSInterruptException:
        logger.error("ROS Interrupted Exception.")
    except Exception as e:
        logger.error(f"An unexpected error occurred: {e}")
