# Q1(a) - ros2_kdl_package

## :package: About

This package contains the tutorial code to create and run your ROS2 C++ node using KDL. It is supposed to be used together with the [ros2_iiwa package].

For **Question 1(a)**, the `ros2_kdl_node` was modified so that the required trajectory and controller variables are loaded as ROS2 parameters from a YAML configuration file.

## :hammer: Build

Clone this package in the `src` folder of your ROS 2 workspace.

Check for missing dependencies:

    rosdep install -i --from-path src --rosdistro humble -y

Build the package:

    colcon build --packages-select ros2_kdl_package

Source the setup files:

    . install/setup.bash

## :white_check_mark: Usage

First, launch the IIWA robot:

    ros2 launch iiwa_bringup iiwa.launch.py

In a new terminal, source the setup files:

    . install/setup.bash

Then launch the `ros2_kdl_node` with the parameters loaded from the YAML file:

    ros2 launch ros2_kdl_package kdl_launch.launch.py

The YAML configuration file is located at:

    config/kdl_params.yaml

It contains the following parameters:

    traj_duration: 1.5
    acc_duration: 0.5
    total_time: 1.5
    trajectory_len: 150
    Kp: 5.0
    end_position_x: 0.5
    end_position_y: 0.0
    end_position_z: 0.5

The launch file:

    launch/kdl_launch.launch.py

automatically loads these parameters and starts the `ros2_kdl_node`.

## :file_folder: Files Added for Q1(a)

The following files were created or modified for Question 1(a):

    config/kdl_params.yaml

    launch/kdl_launch.launch.py

    src/ros2_kdl_node.cpp

The `CMakeLists.txt` was also updated to install the `launch` and `config` directories so that the YAML file and launch file are available after building the package.

## :arrow_forward: Direct Node Execution

The node can also be run directly without the launch file:

    ros2 run ros2_kdl_package ros2_kdl_node

However, for **Question 1(a)**, the recommended method is to use the launch file because it automatically loads the parameters from `config/kdl_params.yaml`.

## :chart_with_upwards_trend: Result

The launch file successfully starts the `ros2_kdl_node` and loads the parameters from the YAML configuration file.

The terminal output confirms that the following values are loaded:

    traj_duration: 1.50
    acc_duration: 0.50
    total_time: 1.50
    trajectory_len: 150
    Kp: 5.00
    end_position: [0.50, 0.00, 0.50]

The node then starts the trajectory execution successfully.

## :white_check_mark: Q 1(b) - Velocity Control with Null-Space Joint-Limit Avoidance

A new velocity controller called `velocity_ctrl_null` has been implemented in the `KDLController` class.

The controller uses a null-space secondary task for joint-limit avoidance. The original velocity controller is kept unchanged so that the two controllers can be selected and compared.

The controller is selected through the ROS2 parameter:

```bash
ctrl:=velocity_ctrl|velocity_ctrl_null
```

The velocity command interface must be used for both controllers.

### Build the package

From the ROS2 workspace:

```bash
cd ~/ros2_ws

colcon build --packages-select ros2_kdl_package

source install/setup.bash
```

### Launch the robot with the velocity interface

Open a first terminal and run:

```bash
cd ~/ros2_ws

source install/setup.bash

ros2 launch iiwa_bringup iiwa.launch.py \
command_interface:="velocity" \
robot_controller:="velocity_controller"
```

Keep this terminal running.

### Run the original velocity controller

Open a second terminal:

```bash
cd ~/ros2_ws

source install/setup.bash

ros2 launch ros2_kdl_package kdl_launch.launch.py \
ctrl:=velocity_ctrl
```

This runs the original velocity controller and can be used as the reference case for comparison.

### Run the null-space velocity controller

Reset/restart the robot before running the second experiment so that the controllers are tested from the same initial configuration.

Then run:

```bash
cd ~/ros2_ws

source install/setup.bash

ros2 launch ros2_kdl_package kdl_launch.launch.py \
ctrl:=velocity_ctrl_null
```

This runs the new velocity controller with the null-space joint-limit avoidance task.

### Controller parameters

The parameters used by the node are loaded from:

```text
ros2_kdl_package/config/kdl_params.yaml
```

For the velocity-controller experiments, the command interface is:

```yaml
cmd_interface: velocity
```

The null-space controller additionally uses the scaling parameter:

```yaml
lambda: 9.9
```

The controller can therefore be switched without modifying the source code:

```bash
ros2 launch ros2_kdl_package kdl_launch.launch.py ctrl:=velocity_ctrl
```

or

```bash
ros2 launch ros2_kdl_package kdl_launch.launch.py ctrl:=velocity_ctrl_null
```

### Verify the selected controller

While the node is running, the selected controller can be checked with:

```bash
ros2 param get /ros2_kdl_node ctrl
```

For the original controller, the expected output is:

```text
String value is: velocity_ctrl
```

For the null-space controller, the expected output is:

```text
String value is: velocity_ctrl_null
```

The velocity command interface can also be checked with:

```bash
ros2 param get /ros2_kdl_node cmd_interface
```

The expected output is:

```text
String value is: velocity
```

### Experimental comparison

The two controllers were compared using:

```text
/velocity_controller/commands
```

for the commanded joint velocities and:

```text
/joint_states
```

for the measured joint positions.

For the tested trajectory, the original `velocity_ctrl` caused Joint 4 to slightly exceed its configured upper limit of `2.09 rad`, reaching approximately `2.1332 rad`.

With `velocity_ctrl_null`, all joints remained inside their configured limits. Joint 4 reached approximately `1.5720 rad`, leaving a margin of approximately `0.5180 rad` from its upper limit.

The commanded velocity and joint-position plots for both controllers are reported in the report.

## Q1(c)

### :dart: Action-based trajectory execution

The `ros2_kdl_node` can be used as a ROS 2 action server to execute the Cartesian trajectory upon request from an action client. The action server is available through

```
/execute_trajectory
```

and uses the custom action interface

```
ros2_kdl_package/action/ExecuteTrajectory
```

The action goal is used to trigger the trajectory execution, while the Cartesian position error norm is continuously published as feedback. At the end of the trajectory, the server returns the execution result.

The action interface is defined as
```
bool start
---
bool success
---
float64 position_error
```

First, launch the robot with the desired command interface. For instance, using the velocity interface
```
ros2 launch iiwa_bringup iiwa.launch.py command_interface:="velocity" robot_controller:="velocity_controller"
```

Then, in a second terminal, launch the KDL node acting as the action server
```
ros2 launch ros2_kdl_package kdl_launch.launch.py cmd_interface:=velocity ctrl:=velocity_ctrl_null
```

The node waits for an action goal before starting the trajectory.

In a third terminal, run the action client
```
ros2 run ros2_kdl_package trajectory_action_client
```

The client sends the trajectory execution goal to the server and displays the position error received as action feedback during the motion. A successful execution produces output similar to
```
Waiting for trajectory action server...
Sending trajectory execution goal
Trajectory goal accepted by server
Position error: ...
Position error: ...
Trajectory executed successfully
```

The available action server can also be checked with
```
ros2 action list
ros2 action info /execute_trajectory
```

## ✅ Q2(a) - ArUco Marker Detection in Gazebo

### 📦 About

For **Question 2(a)**, a custom Gazebo world was created containing:

- the KUKA iiwa robot,
- a static ArUco marker with ID `201`,
- and a standalone simulated camera.

The camera image and camera calibration information are bridged from Gazebo to ROS 2 using `ros_gz_bridge`. The marker is then detected using the `aruco_ros` package.

The detected marker pose is published relative to the simulated camera frame.

---

### 📦 Required Packages

Make sure `aruco_ros` is installed:

```bash
sudo apt install ros-humble-aruco-ros
```

The image viewer used for visual verification can be installed with:

```bash
sudo apt install ros-humble-rqt-image-view
```

---

### 🔨 Build

From the ROS 2 workspace:

```bash
cd ~/ros2_ws
```

Check for missing dependencies:

```bash
rosdep install -i --from-path src --rosdistro humble -y
```

Build the required packages:

```bash
colcon build --packages-select iiwa_description iiwa_bringup
```

Source the setup files:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
```

---

### ▶️ Launch the Gazebo Simulation

Open the first terminal:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

Launch the iiwa robot in simulation:

```bash
ros2 launch iiwa_bringup iiwa.launch.py use_sim:=true
```

The custom Gazebo world contains:

```text
iiwa
arucotag
simulated_camera
ground_plane
sun
```

The ArUco marker is inserted as a static Gazebo model and positioned in front of the simulated camera.

---

### 📷 Launch the ArUco Detection Pipeline

Open a second terminal:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

Run:

```bash
ros2 launch iiwa_description aruco_detection.launch.py
```

The `aruco_detection.launch.py` launch file automatically starts the required Gazebo-to-ROS 2 bridges and the `aruco_ros` detector.

The image pipeline is:

```text
Gazebo /simulated_camera
        ↓
   ros_gz_bridge
        ↓
ROS 2 /simulated_camera
        ↓
     aruco_ros
```

The camera information pipeline is:

```text
Gazebo /camera_info
        ↓
   ros_gz_bridge
        ↓
ROS 2 /camera_info
        ↓
     aruco_ros
```

The detector is configured with:

```text
marker_id: 201
marker_size: 0.1 m
camera_frame: simulated_camera/camera_link/camera
marker_frame: aruco_marker_frame
```

The ROS 2 input topics used by `aruco_ros` are:

```text
/simulated_camera
/camera_info
```

---

### 👁️ Verify the Camera and ArUco Detection

Open another terminal:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

Launch the ROS 2 image viewer:

```bash
ros2 run rqt_image_view rqt_image_view
```

To view the raw image produced by the simulated Gazebo camera, select:

```text
/simulated_camera
```

To view the processed ArUco detection result, select:

```text
/aruco_single/result
```

The processed image shows the detected marker boundary, coordinate axes, and the detected marker ID `201`.

---

### ✅ Verify the Marker Pose

The available ArUco output topics can be checked using:

```bash
ros2 topic list | grep -i aruco
```

The estimated marker pose is published on:

```text
/aruco_single/pose
```

To verify that marker ID `201` is actually being detected, run:

```bash
ros2 topic echo /aruco_single/pose --once
```

A successful detection produces output similar to:

```text
header:
  frame_id: simulated_camera/camera_link/camera

pose:
  position:
    x: -0.0003
    y: -0.0003
    z: 0.3512
  orientation:
    x: 0.7071
    y: 0.7071
    z: 0.00004
    w: -0.00007
```

The detected marker is approximately `0.35 m` in front of the simulated camera, which is consistent with the camera and marker placement in the Gazebo world.

---

### 📁 Files Added or Modified for Q2(a)

The following files were created or modified for **Question 2(a)**:

```text
ros2_iiwa/iiwa_description/gazebo/models/simulated_camera/model.config

ros2_iiwa/iiwa_description/gazebo/models/simulated_camera/model.sdf

ros2_iiwa/iiwa_description/gazebo/worlds/aruco_world.world

ros2_iiwa/iiwa_description/launch/aruco_detection.launch.py

ros2_iiwa/iiwa_description/CMakeLists.txt

ros2_iiwa/iiwa_description/env-hooks/iiwa_description.sh.in

ros2_iiwa/iiwa_bringup/launch/iiwa.launch.py
```

The Gazebo resource paths were updated so that Gazebo can locate the custom models.

The `CMakeLists.txt` file was also updated so that the new `launch` directory is installed with the `iiwa_description` package.

---

### 📈 Result

The custom Gazebo world successfully launches with the KUKA iiwa robot, the ArUco marker, and the simulated camera.

The simulated camera successfully publishes the image and camera information at approximately `30 Hz`.

The `aruco_ros` detector successfully detects marker ID `201`.

The processed detection image is available on:

```text
/aruco_single/result
```

and shows the marker boundary, marker ID, and estimated coordinate axes.

The estimated marker pose is published on:

```text
/aruco_single/pose
```

The successful publication of the marker pose confirms that the ArUco marker is visible to the simulated camera and correctly detected by `aruco_ros`.

This completes **Question 2(a)**.

