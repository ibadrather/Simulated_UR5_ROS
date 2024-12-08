#!/usr/bin/env python

"""
ROS node to save images from a Gazebo camera and log metadata.

Subscribes to '/camera/image_raw' and '/camera/camera_info' topics, saves images to disk,
and logs metadata including filenames, timestamps, camera info, start and end times, and frame rate.

Classes:
    ImageDataSaver: Saves images and logs metadata.

Functions:
    image_callback(msg): Processes incoming image messages.
    camera_info_callback(msg): Processes camera info messages.
    shutdown(): Handles node shutdown and saves metadata.
"""

from pathlib import Path
import rospy
import json
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
from typing import Any, Dict
import uuid
import time

class ImageDataSaver:
    """
    ROS node to save images and log metadata from Gazebo camera.

    Attributes:
        run_id (str): Unique run identifier.
        base_dir (Path): Directory to store images and metadata.
        image_data_dir (Path): Directory for saved images.
        metadata_path (Path): Path to metadata JSON file.
        metadata (dict): Metadata including image data, camera info, and timing.
        bridge (CvBridge): Converter for ROS images to OpenCV.
        image_counter (int): Counter for image filenames.
        last_timestamp (float): Timestamp of the last image.
        frame_rate (float): Frame rate calculated from timestamp differences.
    """
    def __init__(self, base_dir: str = "~/runs") -> None:
        # Setup paths using pathlib
        self.run_id: str = str(uuid.uuid4())
        self.base_dir: Path = Path(base_dir).expanduser()
        self.image_data_dir: Path = self.base_dir / self.run_id / "image_data" / "images"
        self.metadata_path: Path = self.base_dir / self.run_id / "image_data" / "metadata.json"

        # Create directories
        self.image_data_dir.mkdir(parents=True, exist_ok=True)

        # Initialize metadata
        self.metadata: Dict[str, Any] = {
            "run_uuid": self.run_id,
            "camera_info": {},
            "images": [],
            "start_time": time.time(),
            "end_time": None,
            "frame_rate": None
        }
        self.bridge: CvBridge = CvBridge()

        self.image_counter = 0
        self.last_timestamp = None
        self.frame_rate = 0

        # ROS Subscribers
        rospy.Subscriber("/camera/image_raw", Image, self.image_callback)
        rospy.Subscriber("/camera/camera_info", CameraInfo, self.camera_info_callback)

        # Register shutdown callback
        rospy.on_shutdown(self.shutdown)

    def image_callback(self, msg: Image) -> None:
        try:
            # Convert ROS Image to OpenCV format
            cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            timestamp: float = rospy.Time.now().to_sec()

            # Calculate frame rate
            if self.last_timestamp is not None:
                frame_time = timestamp - self.last_timestamp
                if frame_time > 0:
                    self.frame_rate = 1 / frame_time
            self.last_timestamp = timestamp

            # Save image to disk
            filename: str = f"image_{self.image_counter}.png"
            image_path: Path = self.image_data_dir / filename
            cv2.imwrite(str(image_path), cv_img)

            # Append metadata
            self.metadata["images"].append({"filename": filename, "timestamp": timestamp})

            # Save metadata to JSON
            with self.metadata_path.open("w") as f:
                json.dump(self.metadata, f, indent=4)

            self.image_counter += 1

        except Exception as e:
            rospy.logerr(f"Failed to process image: {e}")

    def camera_info_callback(self, msg: CameraInfo) -> None:
        # Save camera info (executed once)
        if not self.metadata["camera_info"]:
            self.metadata["camera_info"] = {
                "width": msg.width,
                "height": msg.height,
                "distortion_model": msg.distortion_model,
                "K": msg.K
            }
            with self.metadata_path.open("w") as f:
                json.dump(self.metadata, f, indent=4)

    def shutdown(self) -> None:
        # Update end time and frame rate in metadata on shutdown
        self.metadata["end_time"] = time.time()
        self.metadata["frame_rate"] = self.frame_rate

        # Save the final metadata
        with self.metadata_path.open("w") as f:
            json.dump(self.metadata, f, indent=4)
        
        rospy.loginfo("Shutdown complete. Metadata saved.")

if __name__ == "__main__":
    run_data_saver_path = Path("/home/edreate/Desktop/Simulated_UR5_ROS/runs")
    
    rospy.init_node("image_data_saver")
    saver = ImageDataSaver(run_data_saver_path)
    rospy.loginfo("Image data saver node started.")
    
    rospy.spin()
