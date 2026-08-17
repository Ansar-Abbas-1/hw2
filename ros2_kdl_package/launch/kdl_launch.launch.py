from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    # Get the package's share directory
    package_share_directory = get_package_share_directory(
        'ros2_kdl_package'
    )

    # Path to the YAML parameter file
    config_file = os.path.join(
        package_share_directory,
        'config',
        'kdl_params.yaml'
    )

    # Launch the ROS 2 KDL node
    kdl_node = Node(
        package='ros2_kdl_package',
        executable='ros2_kdl_node',
        name='ros2_kdl_node',
        output='screen',
        parameters=[config_file]
    )

    return LaunchDescription([
        kdl_node
    ])