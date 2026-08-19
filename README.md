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

