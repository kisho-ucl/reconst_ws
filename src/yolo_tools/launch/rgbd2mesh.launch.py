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
        'param_mesh.yaml'
    )
    return LaunchDescription([
        Node(
            package='yolo_tools',
            namespace='rgbd2mesh',
            executable='rgbd2mesh',
            remappings=[('/input_rgbd', '/camera/camera/rgbd'),
                        ('/output_mesh', '/hoge/mesh_no_color')],
            parameters=[config]
        ),
    ])
