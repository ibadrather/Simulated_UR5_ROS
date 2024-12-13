
## Data Pipeline 

To build a data pipeline for collecting simulation data:
 
1. Create a new package:

```bash
cd src
catkin_create_pkg ur5_data_pipeline rospy sensor_msgs cv_bridge std_msgs
cd ..
catkin_make
source devel/setup.bash
```
 
2. Make scripts executable and run them:

```bash
chmod +x src/ur5_data_pipeline/scripts/image_data_saver.py
rosrun ur5_data_pipeline image_data_saver.py
```
 
3. Set up a virtual environment and install dependencies:

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install src/ur5_data_pipeline/requirements.txt
```
