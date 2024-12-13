# UR5 Robot Simulation with ROS

Simulate and control the Universal Robot UR5 Robot Arm in Gazebo using ROS. This project supports two motion types:

1. **Joint Space Motion**
2. **Linear Space Motion**

---

## Running the Simulation

## Build
Run these commands in the same directory where **src/** folder is located.
```bash
catkin_make
source devel/setup.bash
```

### Launch UR5 Motion Control in Gazebo

To start the simulation and control interface:
```bash
source devel/setup.bash
roslaunch ur5_gazebo_motion_library ur5_motion_control.launch
```

### Run Joint Space Motion API 

Control the robot using joint space motion:


```bash
source devel/setup.bash
rosrun ur5_gazebo_motion_library ur5_joint_motion_API.py
```

### Run Linear Space Motion API 

Control the robot using linear space motion:


```bash
source devel/setup.bash
rosrun ur5_gazebo_motion_library ur5_linear_motion_API.py
```

---


## UR5 Joint Limits 
| Joint Name | Min Position (°) | Max Position (°) | Max Velocity (°/s) | Max Effort (Nm) | 
| --- | --- | --- | --- | --- | 
| Shoulder Pan | -360.0 | 360.0 | 180.0 | 150.0 | 
| Shoulder Lift | -360.0 | 360.0 | 180.0 | 150.0 | 
| Elbow Joint | -180.0 | 180.0 | 180.0 | 150.0 | 
| Wrist 1 | -360.0 | 360.0 | 180.0 | 28.0 | 
| Wrist 2 | -360.0 | 360.0 | 180.0 | 28.0 | 
| Wrist 3 | -360.0 | 360.0 | 180.0 | 28.0 | 


---


## Video 
[Watch Simulation Video on YouTube](https://www.youtube.com/watch?v=hbQc060JycA) 

---


## Media 

### Linear Motion 
![Linear Motion Y-Axis](/media/linear_motion_y_axis.png) 

---

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


---


## Code Formatting 
Ensure code consistency using `black`:

```bash
python3 -m black src/ --line-length 120
```

---

## Setting Up the Environment 
 
1. **Clone the Repository** 
Clone the `universal_robot` package to get the UR5 robot model and its dependencies:

```bash
cd src/
git clone https://github.com/ros-industrial/universal_robot.git
```
 
2. **Build the Workspace** 
Navigate to the root of the Catkin workspace:

```bash
catkin_make
source devel/setup.bash
```
 
3. **Install Dependencies** 
Install required ROS packages:

```bash
rosdep install --from-paths src --ignore-src -r -y
```
Build the workspace again:

```bash
catkin_make
source devel/setup.bash
```


---


## Installation 

### Dependencies 

Install the following dependencies:
 
- `ros-noetic`
 
- `python 3.8`
 
- `black`


```bash
sudo apt-get update
sudo apt-get install ros-noetic-kdl-parser-py
sudo apt-get install ros-noetic-orocos-kdl
sudo apt-get install ros-noetic-python-orocos-kdl
```

### Launch Gazebo with UR5 Robot 

Start the Gazebo simulation:


```bash
roslaunch ur_gazebo ur5_bringup.launch
```


