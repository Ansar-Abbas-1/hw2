from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    # Bridge Gazebo camera image and camera information to ROS 2
    camera_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='camera_bridge',
        output='screen',
        arguments=[
            '/simulated_camera@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
        ],
    )

    # Detect ArUco marker ID 201
    aruco_detector = Node(
        package='aruco_ros',
        executable='single',
        name='aruco_single',
        output='screen',
        parameters=[{
            'image_is_rectified': True,
            'marker_size': 0.1,
            'marker_id': 201,
            'reference_frame': 'simulated_camera/camera_link/camera',
            'camera_frame': 'simulated_camera/camera_link/camera',
            'marker_frame': 'aruco_marker_frame',
            'corner_refinement': 'LINES',
        }],
        remappings=[
            ('/image', '/simulated_camera'),
            ('/camera_info', '/camera_info'),
        ],
    )

    return LaunchDescription([
        camera_bridge,
        aruco_detector,
    ])
