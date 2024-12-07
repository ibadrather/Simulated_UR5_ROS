# UR5 Robot Simulation with ROS

Simulate Universal Robot UR5 Robot Arm in Gazebo and control using ROS.

To start:

```bash
mkdir src
cd src
catkin_init_workspace
cd ..
catkin_make
```

1. Clone the `universal_robot` repositoryClone the `universal_robot` package to get the UR5 robot model and its dependencies for Gazebo:

```bash
cd src/
git clone https://github.com/ros-industrial/universal_robot.git
```

This provides the UR5 robot model and control files for Gazebo.

### 2. Install dependencies and build the workspace 

Navigate to your Catkin workspace (root of this repository) and install all dependencies:


```bash
catkin_make
source devel/setup.bash
```

Install dependencies:


```bash
rosdep install --from-paths src --ignore-src -r -y
```

Build the workspace:


```bash
catkin_make
source devel/setup.bash
```

### 3. Launch Gazebo with the UR5 robot 

After building the workspace, launch the UR5 robot in Gazebo:


```bash
roslaunch ur_gazebo ur5_bringup.launch
```

This opens the Gazebo simulation environment with the UR5 robot model.

### 4. Created a Custom ROS Node to Publish Joint Angles 
Created a new package to write a custom node (`ur5_sim_control`):

```bash
cd src/
catkin_create_pkg ur5_sim_control roscpp std_msgs
```

Build the package:


```bash
catkin_make
source devel/setup.bash
```

Launch the custom sine wave control node:


```bash
roslaunch ur5_sim_control ur5_sine_wave_control.launch
```

---

### UR5 Joint Limits

| Joint Name | Min Position (°) | Max Position (°) | Max Velocity (°/s) | Max Effort (Nm) | 
| --- | --- | --- | --- | --- | 
| Shoulder Pan | -360.0 | 360.0 | 180.0 | 150.0 | 
| Shoulder Lift | -360.0 | 360.0 | 180.0 | 150.0 | 
| Elbow Joint | -180.0 | 180.0 | 180.0 | 150.0 | 
| Wrist 1 | -360.0 | 360.0 | 180.0 | 28.0 | 
| Wrist 2 | -360.0 | 360.0 | 180.0 | 28.0 | 
| Wrist 3 | -360.0 | 360.0 | 180.0 | 28.0 | 


---