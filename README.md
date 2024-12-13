# UR5 Robot Simulation with ROS

Simulate and control the Universal Robot UR5 Robot Arm in Gazebo using ROS. This project supports three motion types:

1. **Sinusoidal Motion** 
2. **Joint Space Motion**
3. **Linear Space Motion**

#### For detailed documentation on API usage, go to the API.md here: [API.md](./API.md)

---

## Running the Simulation

### Build the Workspace
Run these commands in the root directory of your Catkin workspace (where the **src/** folder is located):
```bash
catkin_make
source devel/setup.bash
```

### Launch UR5 Sine Movement

To simulate sinusoidal motion for the UR5:
```bash
source devel/setup.bash
roslaunch ur5_sim_control ur5_sine_wave_control.launch
```

##### Package Structure

```bash
src/ur5_sim_control/
├── CMakeLists.txt
├── include
│   └── ur5_sim_control
├── launch
│   ├── custom_ur5_bringup.launch
│   └── ur5_sine_wave_control.launch
├── package.xml
├── rviz
│   └── ur5_sim_control.rviz
├── src
│   └── sine_wave_control.cpp
└── worlds
    ├── ur5_in_a_room.world
    └── walls
        ├── model.config
        └── model.sdf
```

### Launch UR5 Motion Control in Gazebo 

To initialize the UR5 simulation and motion control interface in Gazebo, run the following commands:


```bash
source devel/setup.bash
roslaunch ur5_gazebo_motion_library ur5_motion_control.launch
```

This will launch the Gazebo environment with the UR5 robot model, ready for motion control tasks.


---


### Run the Linear Space Motion API 

To control the UR5 robot in Cartesian space (linear motion), use the following command:


```bash
source devel/setup.bash
rosrun ur5_gazebo_motion_library ur5_linear_motion_API.py
```
**Updating Input and Output Poses** 
Modify the start and end poses directly in the script to specify the desired Cartesian trajectory:File: `src/ur5_gazebo_motion_library/scripts/ur5_linear_motion_API.py`

```python
start_pose: Pose = create_pose(0.3, 0.6, 0.1, 1.0, 0.0, 0.0, 0.0)
end_pose: Pose = create_pose(0.3, -0.6, 0.1, 1.0, 0.0, 0.0, 0.0)
```
For more detailed information on how to use the Linear Space Motion API, refer to the [API.md](./API.md)  file.

---


### Run the Joint Space Motion API 

To control the UR5 robot in joint space, execute the following:


```bash
source devel/setup.bash
rosrun ur5_gazebo_motion_library ur5_joint_motion_API.py
```
**Customizing Joint Configurations** 
Define the desired joint positions by modifying the script:File: `src/ur5_gazebo_motion_library/scripts/ur5_joint_motion_API.py`

```python
# Retrieve current joint positions
current_joints = ur5_api.get_current_joint_positions()

# Define a target joint configuration (e.g., increment each joint by 0.5 radians)
target_joints = [cj + 0.5 for cj in current_joints]
```

For comprehensive instructions and examples on using the Joint Space Motion API, consult the [API.md](./API.md)  file.


---

## Media

### Sinusoidal Motion Video

- [Watch UR5 Sinusoidal Movement Video on YouTube](https://www.youtube.com/watch?v=zaZnu9Zkfec)

#### Sinusoidal Motion Plot

![Sinusoidal Motion](/media/ur5_joint_sinusoidal.png)

### Linear Motion Simulation Video

- [Watch Simulation Video on YouTube](https://www.youtube.com/watch?v=hbQc060JycA)

#### Linear Motion Plot

The End Effector moves along Y axis with a linear motion ([Watch on YouTube](https://www.youtube.com/watch?v=hbQc060JycA)).

![Linear Motion Y-Axis](/media/linear_motion_y_axis.png)

---

## Installation

### Dependencies

Install the following dependencies:

- `ros-noetic`
- `python 3.8`
- `black`

Use the following commands to install ROS-related dependencies:
```bash
sudo apt-get update
sudo apt-get install ros-noetic-kdl-parser-py
sudo apt-get install ros-noetic-orocos-kdl
sudo apt-get install ros-noetic-python-orocos-kdl
```

## Code Formatting

Ensure code consistency using `black`:
```bash
python3 -m black src/ --line-length 120
```

---

## Setting Up the Environment

### Clone the Repository

Clone the `universal_robot` package to get the UR5 robot model and its dependencies:
```bash
cd src/
git clone https://github.com/ros-industrial/universal_robot.git
```

### Build the Workspace

Navigate to the root of the Catkin workspace and build:
```bash
catkin_make
source devel/setup.bash
```

### Install Dependencies

Install required ROS packages:
```bash
rosdep install --from-paths src --ignore-src -r -y
```
Rebuild the workspace:
```bash
catkin_make
source devel/setup.bash
```

---
