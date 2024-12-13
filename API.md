# UR5MotionAPI Documentation 
This documentation provides detailed instructions on using the `UR5JointMotionAPI` and `UR5LinearMotionAPI` classes to control the UR5 robot in both joint and Cartesian spaces.

---

## UR5LinearMotionAPI Class 
The `UR5LinearMotionAPI` class provides functionality to plan and execute Cartesian space trajectories for the UR5 robot.
### Key Methods 
 
1. **`get_current_joint_positions()`**  
  - **Description:**  Retrieves the current joint positions of the UR5 robot.
 
  - **Usage:** 

```python
current_joints = ur5_api.get_current_joint_positions()
```
 
2. **`generate_cartesian_trajectory(start_pose, end_pose, velocity, acceleration, steps)`**  
  - **Description:**  Creates a Cartesian trajectory moving the robot's end-effector from a start pose to an end pose.
 
  - **Parameters:**  
    - `start_pose` (Pose): Starting pose of the end effector.
 
    - `end_pose` (Pose): Target pose of the end effector.
 
    - `velocity` (float): Desired linear velocity (m/s).
 
    - `acceleration` (float): Desired linear acceleration (m/s²).
 
    - `steps` (int): Number of interpolation steps for the trajectory.
 
  - **Returns:**  ROS `JointTrajectory` message.
 
  - **Usage:** 

```python
trajectory_msg = ur5_api.generate_cartesian_trajectory(
    start_pose, end_pose, linear_vel=0.5, linear_acc=0.1, steps=200
)
```
 
3. **`execute_trajectory(trajectory_msg)`**  
  - **Description:**  Executes a Cartesian trajectory generated by the API.
 
  - **Parameters:**  
    - `trajectory_msg`: Trajectory message generated by `generate_cartesian_trajectory`.
 
  - **Usage:** 

```python
ur5_api.execute_trajectory(trajectory_msg)
```

### Workflow Summary 
 
1. **Initialize**  the `UR5LinearMotionAPI` class.
 
2. **Retrieve the current joint positions**  using `get_current_joint_positions()`.
 
3. **Define a start and end pose**  for the Cartesian motion.
 
4. **Generate a Cartesian trajectory**  with `generate_cartesian_trajectory()`.
 
5. **Execute the trajectory**  using `execute_trajectory()`.

### Full Workflow Example 


```python
from ur5_motion_api import UR5LinearMotionAPI
from geometry_msgs.msg import Pose

def create_pose(x, y, z, qx, qy, qz, qw):
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z
    pose.orientation.x = qx
    pose.orientation.y = qy
    pose.orientation.z = qz
    pose.orientation.w = qw
    return pose

# Initialize the API
ur5_api = UR5LinearMotionAPI()

# Retrieve current joint positions
current_joints = ur5_api.get_current_joint_positions()

# Define start and end poses for Cartesian motion
start_pose = create_pose(0.3, 0.6, 0.1, 1.0, 0.0, 0.0, 0.0)
end_pose = create_pose(0.3, -0.6, 0.1, 1.0, 0.0, 0.0, 0.0)

# Define velocity and acceleration for Cartesian motion
linear_vel = 0.5  # m/s
linear_acc = 0.1  # m/s²

# Generate the Cartesian trajectory
trajectory_msg = ur5_api.generate_cartesian_trajectory(
    start_pose=start_pose,
    end_pose=end_pose,
    velocity=linear_vel,
    acceleration=linear_acc,
    steps=200
)

# Execute the trajectory if generation was successful
if trajectory_msg is not None:
    ur5_api.execute_trajectory(trajectory_msg)
else:
    rospy.logerr("Failed to generate Cartesian trajectory.")
```

---

## UR5JointMotionAPI Class 
The `UR5JointMotionAPI` class offers high-level methods to control the UR5 robot in joint space.
### Key Methods 
 
1. **`get_current_joint_positions()`**  
  - **Description:**  Retrieves the current joint angles of the UR5 robot.
 
  - **Usage:** 

```python
current_joints = ur5_api.get_current_joint_positions()
```
 
2. **`generate_joint_space_trajectory(start_positions, target_positions, velocity, acceleration, steps)`**  
  - **Description:**  Generates a trajectory to move the robot from current joint positions to target joint positions.
 
  - **Parameters:**  
    - `start_positions` (List[float]): Current joint angles.
 
    - `target_positions` (List[float]): Desired joint angles.
 
    - `velocity` (float): Desired joint velocity (rad/s).
 
    - `acceleration` (float): Desired joint acceleration (rad/s²).
 
    - `steps` (int): Number of interpolation points in the trajectory.
 
  - **Returns:**  ROS-compatible trajectory message.
 
  - **Usage:** 

```python
trajectory_msg = ur5_api.generate_joint_space_trajectory(
    current_joints, target_joints, joint_vel, joint_acc, steps=100
)
```
 
3. **`execute_trajectory(trajectory_msg)`**  
  - **Description:**  Executes a precomputed trajectory on the UR5 robot.
 
  - **Parameters:**  
    - `trajectory_msg`: Trajectory message generated by `generate_joint_space_trajectory`.
 
  - **Usage:** 

```python
ur5_api.execute_trajectory(trajectory_msg)
```

### Workflow Summary 
 
1. **Initialize**  the `UR5JointMotionAPI` class.
 
2. **Retrieve current joint positions**  using `get_current_joint_positions()`.
 
3. **Define a target configuration**  and generate a trajectory using `generate_joint_space_trajectory()`.
 
4. **Execute the trajectory**  with `execute_trajectory()`.

### Full Workflow Example 


```python
from ur5_motion_api import UR5JointMotionAPI

# Initialize the API
ur5_api = UR5JointMotionAPI()

# Retrieve current joint positions
current_joints = ur5_api.get_current_joint_positions()

# Define target joint positions (e.g., increment each joint by 0.5 radians)
target_joints = [cj + 0.5 for cj in current_joints]

# Generate the joint space trajectory
trajectory_msg = ur5_api.generate_joint_space_trajectory(
    start_positions=current_joints,
    target_positions=target_joints,
    velocity=0.5,      # rad/s
    acceleration=0.2,  # rad/s²
    steps=100
)

# Execute the trajectory if generation was successful
if trajectory_msg is not None:
    ur5_api.execute_trajectory(trajectory_msg)
else:
    rospy.logerr("Failed to generate joint space trajectory.")
```

---


## Summary 

This documentation covers two primary APIs for controlling the UR5 robot:
 
- **UR5JointMotionAPI:**  For joint space control, including retrieving joint positions, generating joint trajectories, and executing them.
 
- **UR5LinearMotionAPI:**  For Cartesian space control, including retrieving joint positions, generating Cartesian trajectories based on poses, and executing them.

By following the provided methods and workflow examples, users can effectively control the UR5 robot for various motion planning and execution tasks.
