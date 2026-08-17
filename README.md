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

