# 📘 Homework 2

This repository contains the implementation and experimental results for the different parts of the homework using **ROS 2 Humble**, the **KUKA iiwa**, **KDL**, and **Gazebo/Ignition**.

The homework was developed incrementally. To keep the implementation of each question separate and reproducible, the different question parts are stored in dedicated Git branches.

---

## 🚀 How to Use This Repository

The workspace uses the following two main ROS 2 packages:

```text
ros2_iiwa
ros2_kdl_package
```

Clone both packages into the `src` directory of a ROS 2 workspace.

A typical workspace structure is:

```text
ros2_ws/
├── src/
│   ├── ros2_iiwa/
│   └── ros2_kdl_package/
├── build/
├── install/
└── log/
```

After cloning the packages, build the workspace:

```bash
cd ~/ros2_ws

source /opt/ros/humble/setup.bash

colcon build

source install/setup.bash
```

---

## 🌿 Homework Organization Using Git Branches

Each question or question part is stored in a separate Git branch.

This makes it possible to inspect and test the implementation corresponding to a specific part of the homework without mixing it with changes introduced in later parts.

The branch organization is:

```text
main        → Question 1(a)
q1_part_b   → Question 1(b)
q1_part_c   → Question 1(c)
q2-part-a   → Question 2(a)
q2-part-b   → Question 2(b)
q2-part-c   → Question 2(c)
```

**Question 1(a)** is implemented on the `main` branch.

For all subsequent parts, the branch name corresponds directly to the question and part being tested.

---

## 🔄 Switching to a Specific Homework Part

Before testing a particular question, switch to its corresponding branch.

For example, to test **Question 2(b)**:

```bash
cd ~/ros2_ws/src
cd <repository-directory>

git checkout q2-part-b
```

To test **Question 2(c)**:

```bash
git checkout q2-part-c
```

To return to **Question 1(a)**:

```bash
git checkout main
```

You can check the currently selected branch using:

```bash
git branch
```

The active branch is indicated by `*`.

For example:

```text
  main
  q1_part_b
  q1_part_c
  q2-part-a
* q2-part-b
  q2-part-c
```

In this example, the implementation for **Question 2(b)** is currently selected.

---

## 🔨 Rebuild After Switching Branches

After switching to another homework branch, rebuild the workspace so that the executable files correspond to the selected implementation.

From the workspace root:

```bash
cd ~/ros2_ws

source /opt/ros/humble/setup.bash

colcon build

source install/setup.bash
```

If only the packages used in this homework need to be rebuilt, they can also be built selectively:

```bash
colcon build --packages-select iiwa_description ros2_kdl_package
```

Then source the workspace again:

```bash
source install/setup.bash
```

---

## 🧪 Testing a Specific Question

Each homework part contains its own instructions in the corresponding branch's `README.md`.

The recommended procedure is:

1. Switch to the branch corresponding to the question.
2. Build the ROS 2 workspace.
3. Source the ROS 2 and workspace setup files.
4. Read the question-specific section of the `README.md`.
5. Open the required terminals described in that section.
6. Run the provided launch commands.
7. Perform the specified test or experiment.
8. Compare the observed behaviour with the expected result documented in the README.

For example, to verify **Question 2(c)**:

```bash
git checkout q2-part-c

cd ~/ros2_ws
source /opt/ros/humble/setup.bash

colcon build

source install/setup.bash
```

Then follow the **Q2(c)** section of the README to launch Gazebo, start the ArUco detection pipeline and service bridge, and test the marker pose update using the provided ROS 2 service call.

---

## 📖 Question-Specific Instructions

The README in each branch documents the commands required to reproduce that part of the homework.

These instructions include, where applicable:

- required ROS 2 launch commands,
- controller selection,
- command interface selection,
- ROS 2 parameters,
- required topics,
- ROS 2 services,
- Gazebo/Ignition interfaces,
- verification commands,
- expected behaviour,
- safety behaviour,
- ROS 2 bag recording,
- plots and experimental results.

Therefore, after switching to the required branch, follow the corresponding question section in the README to reproduce and verify the implementation.

---

## ✅ Reproducing the Homework

In summary, the workflow for reproducing any homework part is:

```text
Clone packages
      ↓
Create/build ROS 2 workspace
      ↓
Switch to the required Git branch
      ↓
Rebuild and source the workspace
      ↓
Read the corresponding README section
      ↓
Run the documented launch commands
      ↓
Perform the specified test
      ↓
Verify the expected result
```

This branch-based organization keeps each homework stage reproducible and allows the implementation of every question to be inspected and tested independently.


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


## ✅ Q2(b) - Vision-Based Look-at-Point Control

### 📦 About

For **Question 2(b)**, a new vision-based velocity controller called `vision_ctrl` was implemented in the `KDLController` class.

The controller performs a **look-at-point task** using the pose of the ArUco marker detected by `aruco_ros`.

The objective is to continuously orient the eye-in-hand camera towards the ArUco marker while the marker is manually moved in Gazebo.

The controller uses:

- the normalized direction from the camera to the ArUco marker,
- the current camera rotation,
- the camera Jacobian,
- the visual interaction matrix,
- and a null-space joint-limit avoidance term.

The controller is selected using:

```bash
ctrl:=vision
```

and must be used with the velocity command interface.

---

### 🔨 Build

From the ROS 2 workspace:

```bash
cd ~/ros2_ws
```

Build the required package:

```bash
colcon build --packages-select ros2_kdl_package
```

Source the setup files:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
```

---

### ▶️ Launch the Robot

Open the first terminal:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

Launch the iiwa robot in Gazebo using the velocity controller:

```bash
ros2 launch iiwa_bringup iiwa.launch.py \
  use_sim:=true \
  command_interface:="velocity" \
  robot_controller:="velocity_controller"
```

Keep this terminal running.

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

This starts the eye-in-hand camera bridge and the `aruco_ros` detector.

The detected marker pose is published on:

```text
/aruco_single/pose
```

The `ros2_kdl_node` subscribes to this topic and uses the marker position for the visual controller.

---

### 🎯 Run the Vision Controller

Open a third terminal:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

Launch the KDL node with the velocity interface and the vision controller:

```bash
ros2 launch ros2_kdl_package kdl_launch.launch.py \
cmd_interface:=velocity \
ctrl:=vision
```
The selected controller can be verified with:

```bash
ros2 param get /ros2_kdl_node ctrl
```

The expected output is:

```text
String value is: vision
```

The velocity command interface can be checked using:

```bash
ros2 param get /ros2_kdl_node cmd_interface
```

The expected output is:

```text
String value is: velocity
```

---

### ⚙️ Controller Parameters

The controller parameters are loaded from:

```text
ros2_kdl_package/config/kdl_params.yaml
```

The parameters used for the final experiment were:

```yaml
cmd_interface: velocity

Kp: 0.2
lambda: 500.9
```

`Kp` controls the magnitude of the primary visual-control command.

`lambda` scales the secondary joint-limit avoidance motion.

A larger value of `lambda` was used so that the null-space contribution remains small compared with the primary visual task.

For safety, the commanded joint velocities are also saturated to:

```text
±0.05 rad/s
```

before being published to the velocity controller.

---

### 👁️ Visual Task

The ArUco marker position is received from:

```text
/aruco_single/pose
```

and expressed in the camera optical frame.

The marker position vector is normalized to obtain the current viewing direction `s`.

The desired viewing direction is:

```text
sd = [0, 0, 1]
```

which corresponds to the positive optical z-axis of the camera.

The visual interaction matrix is constructed using:

- the current viewing direction,
- the distance between the camera and the marker,
- and the current camera rotation.

The interaction matrix is then combined with the camera Jacobian to obtain the visual Jacobian used by the controller.

The final commanded joint velocity contains two contributions:

```text
primary vision task
+
null-space joint-limit avoidance task
```

The primary task generates the robot motion required to orient the camera towards the ArUco marker.

The null-space projector allows the robot to perform secondary joint-limit avoidance motion without interfering with the primary visual tracking task.

---

### 🦾 Camera Jacobian and Eye-in-Hand Configuration

For the vision controller, the KDL end-effector is configured to correspond to the eye-in-hand camera frame rather than the original robot tool frame.

The camera Jacobian therefore describes the linear and angular motion of the camera with respect to the robot joints.

The controller uses a `6 x 7` camera Jacobian:

```text
Jc: 6 x 7
```

The six rows correspond to the three linear and three angular velocity components of the camera, while the seven columns correspond to the seven joints of the KUKA iiwa.

This camera Jacobian is different from the original end-effector Jacobian and is used specifically for the vision-based task.

---

### 🛡️ Safety Behaviour

Two safety mechanisms are included in the vision-control implementation.

#### ArUco Pose Timeout

If no fresh ArUco marker pose is received for more than approximately `0.5 s`, the controller immediately publishes zero joint velocities.

A warning similar to the following is produced:

```text
ArUco marker lost/stale (...) Sending zero velocity.
```

The commanded velocity becomes:

```text
[0, 0, 0, 0, 0, 0, 0]
```

This prevents the robot from continuing to execute an old velocity command when the marker is no longer detected or when the ArUco pose stream stops.

#### Safe Shutdown

A separate shutdown protection is also implemented.

When the KDL node is stopped using `Ctrl+C`, the command timer is cancelled and zero joint velocities are sent before the node terminates.

Typical output is:

```text
Command timer cancelled.
Stopping robot: sending ZERO joint velocities.
```

This prevents the velocity controller from continuing to execute the last non-zero command after the KDL node is stopped.

---

### 📊 Record the Experiment with ROS 2 Bag

The final visual-tracking experiment was recorded using ROS 2 bag.

Create a directory for the recordings:

```bash
cd ~/ros2_ws
mkdir -p bags
cd bags
```

Record the commanded joint velocities, measured joint states, and ArUco pose:

```bash
ros2 bag record \
/velocity_controller/commands \
/joint_states \
/aruco_single/pose \
-o vision_tracking
```

During recording, manually move the ArUco marker to several different positions using the Gazebo interface.

The robot should continuously reorient the eye-in-hand camera and follow the marker.

Stop the recording using:

```text
Ctrl+C
```

The recorded bag can be checked using:

```bash
ros2 bag info ~/ros2_ws/bags/vision_tracking
```

The final experiment contained the following topics:

```text
/velocity_controller/commands
/aruco_single/pose
/joint_states
```

The recorded experiment had a duration of approximately:

```text
145 s
```

The bag contained approximately:

```text
1447   velocity command messages
4359   ArUco pose messages
13197  joint state messages
```

---

### 📈 Velocity Command Plot

The topic used for the required homework plot is:

```text
/velocity_controller/commands
```

This topic contains the commanded velocities of all seven robot joints.

The recorded commands were extracted from the ROS 2 bag and plotted against time.

The plot contains the seven commanded joint velocities:

```text
qdot1
qdot2
qdot3
qdot4
qdot5
qdot6
qdot7
```

with:

```text
x-axis: Time [s]
y-axis: Commanded joint velocity [rad/s]
```

The resulting plot shows that manually changing the position of the ArUco marker produces corresponding changes in the commanded robot joint velocities.

After each marker displacement, the visual controller generates a new combination of joint velocities to reorient the eye-in-hand camera towards the marker.

As the robot approaches the required viewing direction, the commanded joint velocities generally reduce in magnitude.

The different signs and magnitudes of the seven joint velocities are expected because the visual Jacobian and its pseudoinverse distribute the required camera motion across the redundant seven-joint manipulator.

The commanded velocities remained well below the imposed safety saturation:

```text
±0.05 rad/s
```

throughout the experiment.

---

### ✅ Result

The `vision_ctrl` controller successfully performs the required look-at-point visual tracking task.

The robot continuously follows the manually moved ArUco marker by changing the pose and orientation of the eye-in-hand camera.

The controller successfully combines:

- the ArUco marker position,
- the normalized viewing direction,
- the visual interaction matrix,
- the camera Jacobian,
- the pseudoinverse solution,
- and the null-space joint-limit avoidance contribution.

The null-space motion remains secondary to the main vision task and allows redundant robot motion to be used for joint-limit avoidance.

The commanded joint velocities were recorded using ROS 2 bag and plotted over the complete experiment.

The resulting velocity plot demonstrates the response of the controller to repeated manual repositioning of the ArUco marker in Gazebo.

Marker-loss protection and safe shutdown behaviour were also verified by confirming that zero joint velocities are published whenever fresh ArUco data is unavailable or the KDL node is terminated.

This completes **Question 2(b)**.


## ✅ Q2(c) - ROS 2 Service for Updating the ArUco Marker Pose

### 📦 About

For **Question 2(c)**, the existing Gazebo pose service was bridged to ROS 2 so that the position of the ArUco marker can be changed directly using a ROS 2 service call.

The Gazebo service used is:

```text
/world/default/set_pose
```

The corresponding ROS 2 service interface is:

```text
ros_gz_interfaces/srv/SetEntityPose
```

A `parameter_bridge` was added to the previously created:

```text
iiwa_description/launch/aruco_detection.launch.py
```

This allows the Gazebo `set_pose` service to be accessed directly from ROS 2.

The ArUco marker is represented in Gazebo by the model:

```text
arucotag
```

---

### 🔗 Service Bridge

The following service bridge is created inside `aruco_detection.launch.py`:

```python
set_pose_bridge = Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    name='set_pose_bridge',
    output='screen',
    arguments=[
        '/world/default/set_pose@ros_gz_interfaces/srv/SetEntityPose',
    ],
)
```

The bridge node is included in the launch description together with the camera bridge and ArUco detector:

```python
return LaunchDescription([
    camera_bridge,
    set_pose_bridge,
    aruco_detector,
])
```

Therefore, the `set_pose` bridge is started automatically when the ArUco detection launch file is executed.

A separate manual `parameter_bridge` command is not required.

---

### 🔨 Build

From the ROS 2 workspace:

```bash
cd ~/ros2_ws
```

Build the modified package:

```bash
colcon build --packages-select iiwa_description
```

Source ROS 2 and the workspace:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

---

### ▶️ Terminal 1 - Launch the IIWA Robot in Gazebo

Open the first terminal and source the workspace:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

Launch the IIWA simulation:

```bash
ros2 launch iiwa_bringup iiwa.launch.py \
  use_sim:=true \
  command_interface:="velocity" \
  robot_controller:="velocity_controller"
```

Keep this terminal running.

The Gazebo world contains the IIWA robot and the `arucotag` model used as the visual target.

---

### 📷 Terminal 2 - Launch ArUco Detection and the Set-Pose Bridge

Open a second terminal:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

Launch:

```bash
ros2 launch iiwa_description aruco_detection.launch.py
```

This launch file starts:

- the eye-in-hand camera bridge,
- the `set_pose_bridge`,
- and the `aruco_ros` detector.

The `set_pose_bridge` automatically exposes the Gazebo service:

```text
/world/default/set_pose
```

to ROS 2.

Keep this terminal running.

---

### 🔍 Terminal 3 - Verify the Bridged Service

Open a third terminal:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

Check that the service is available:

```bash
ros2 service list | grep set_pose
```

The important expected service is:

```text
/world/default/set_pose
```

ROS 2 may also show the normal parameter services associated with the `set_pose_bridge` node, such as:

```text
/set_pose_bridge/describe_parameters
/set_pose_bridge/get_parameter_types
/set_pose_bridge/get_parameters
/set_pose_bridge/list_parameters
/set_pose_bridge/set_parameters
/set_pose_bridge/set_parameters_atomically
```

These are normal ROS 2 parameter services.

Verify the type of the bridged Gazebo service:

```bash
ros2 service type /world/default/set_pose
```

The expected result is:

```text
ros_gz_interfaces/srv/SetEntityPose
```

---

### 🎯 Terminal 3 - Move the ArUco Marker Using a ROS 2 Service Call

The ArUco marker can now be repositioned directly from ROS 2.

For example:

```bash
ros2 service call /world/default/set_pose \
ros_gz_interfaces/srv/SetEntityPose \
"{entity: {name: 'arucotag', type: 2}, pose: {position: {x: 0.5, y: 0.2, z: 0.5}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
```

Here:

```text
name: 'arucotag'
```

selects the ArUco marker model.

The value:

```text
type: 2
```

specifies that the selected Gazebo entity is a model.

The requested position in this example is:

```text
x = 0.5 m
y = 0.2 m
z = 0.5 m
```

The orientation is specified using the quaternion:

```text
x = 0.0
y = 0.0
z = 0.0
w = 1.0
```

A successful request should return:

```text
response:
ros_gz_interfaces.srv.SetEntityPose_Response(success=True)
```

The `arucotag` model should also visibly move to the requested position in Gazebo.

---

### 🔄 Test with Another Marker Position

The service can be called again with another pose.

For example:

```bash
ros2 service call /world/default/set_pose \
ros_gz_interfaces/srv/SetEntityPose \
"{entity: {name: 'arucotag', type: 2}, pose: {position: {x: 0.6, y: -0.2, z: 0.6}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
```

Again, a successful call should return:

```text
success=True
```

and the marker should move to the newly requested position in Gazebo.

---

### 👁️ Optional Combined Test with Q2(b) Vision Controller

The service implemented in Q2(c) can also be tested together with the vision controller developed in **Q2(b)**.

Keep **Terminal 1** and **Terminal 2** running.

Open another terminal:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

Launch the vision controller:

```bash
ros2 launch ros2_kdl_package kdl_launch.launch.py \
  cmd_interface:=velocity \
  ctrl:=vision
```

The complete system can then be tested by calling the `set_pose` service from another terminal.

A moderate marker displacement can be requested, for example:

```bash
ros2 service call /world/default/set_pose \
ros_gz_interfaces/srv/SetEntityPose \
"{entity: {name: 'arucotag', type: 2}, pose: {position: {x: 0.5, y: 0.15, z: 0.5}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
```

When the marker remains inside the camera field of view:

1. Gazebo moves the `arucotag` model to the requested pose.
2. The eye-in-hand camera observes the marker at its new location.
3. `aruco_ros` updates `/aruco_single/pose`.
4. The `vision_ctrl` controller receives the new marker direction.
5. New joint velocity commands are calculated.
6. The robot reorients the eye-in-hand camera towards the marker.

Small or moderate marker displacements are preferable for this combined test so that the marker remains visible to the camera.

---

### 🛡️ Behaviour if the Marker Leaves the Camera View

If the requested marker position places the ArUco marker outside the camera field of view, the ArUco detector will no longer provide fresh marker measurements.

The safety mechanism implemented for the vision controller then detects the stale marker measurement and commands zero joint velocities.

Therefore, losing the marker during a large pose change does not indicate a failure of the `set_pose` service. The service may have moved the marker successfully, while the vision controller correctly stops because the visual target is no longer visible.

---

### 🔧 Important Components Used

The Q2(c) implementation uses the following ROS 2 and Gazebo components:

- `ros_gz_bridge`
- `parameter_bridge`
- `ros_gz_interfaces/srv/SetEntityPose`
- `/world/default/set_pose`
- `aruco_detection.launch.py`
- `arucotag` Gazebo model
- `aruco_ros`

The bridge connects the existing Gazebo pose-setting functionality to ROS 2, allowing the marker pose to be controlled through the standard ROS 2 service interface.

---

### ✅ Result

The Gazebo `/world/default/set_pose` service was successfully exposed to ROS 2 using a `ros_gz_bridge` `parameter_bridge`.

The bridge was added directly to:

```text
iiwa_description/launch/aruco_detection.launch.py
```

and therefore starts automatically with the ArUco detection pipeline.

The bridged service was verified using:

```bash
ros2 service list | grep set_pose
```

and its ROS 2 type was confirmed as:

```text
ros_gz_interfaces/srv/SetEntityPose
```

The service was then tested using `ros2 service call`.

The calls returned:

```text
success=True
```

and the `arucotag` model visibly moved to the requested positions in Gazebo.

The service can also be used together with the Q2(b) vision controller, allowing the ArUco target to be repositioned from ROS 2 while the eye-in-hand vision controller reacts to the updated target location.

This completes **Question 2(c)**.

