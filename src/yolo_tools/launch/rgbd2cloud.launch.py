import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('yolo_tools'),
        'config',
        'param.yaml'
    )
    return LaunchDescription([
        Node(
            package='yolo_tools',
            namespace='rgbd2cloud',
            executable='rgbd2cloud',
            remappings=[('/input_rgbd', '/camera/rgbd'),
                        ('/output_cloud', '/hoge/cloud'),
                        ('/output_image', '/hoge/maks')],
            parameters=[config]
        ),
    ])
