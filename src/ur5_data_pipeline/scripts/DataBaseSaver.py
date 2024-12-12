import rospy
from sqlalchemy import create_engine, Column, Integer, String, Float
from sqlalchemy.orm import sessionmaker, declarative_base
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

    __tablename__ = "joint_states"

    id = Column(Integer, primary_key=True, autoincrement=True)
    joint_name = Column(String)
    position = Column(Float)
    velocity = Column(Float)
    effort = Column(Float)
    timestamp = Column(Float)


class DatabaseSaver:
    def __init__(self, sqlite_db_path: str, run_uuid: str, use_influxdb=False):
        self.use_influxdb = use_influxdb
        self.run_uuid = run_uuid
        if self.use_influxdb:
            self.client = InfluxDBClient3(host=HOST, token=TOKEN, org=ORG)
            self.bucket_name = BUCKET_NAME
        else:
            # Set up SQLite using SQLAlchemy
            self.engine = create_engine(f"sqlite:///{sqlite_db_path}", echo=True)
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
                        timestamp=timestamp,
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
