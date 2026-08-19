from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

import os


def generate_launch_description():

    # Controller selection
    ctrl_arg = DeclareLaunchArgument(
        'ctrl',
        default_value='velocity_ctrl',
        description='Velocity controller: velocity_ctrl or velocity_ctrl_null'
    )

    ctrl = LaunchConfiguration('ctrl')

    # Get package share directory
    package_share_directory = get_package_share_directory(
        'ros2_kdl_package'
    )

    # Path to YAML parameter file
    config_file = os.path.join(
        package_share_directory,
        'config',
        'kdl_params.yaml'
    )

    # ROS2 KDL node
    kdl_node = Node(
        package='ros2_kdl_package',
        executable='ros2_kdl_node',
        name='ros2_kdl_node',
        output='screen',
        parameters=[
            config_file,
            {'ctrl': ctrl}
        ]
    )

    return LaunchDescription([
        ctrl_arg,
        kdl_node
    ])
