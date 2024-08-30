from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='calibration_tool',
            executable='find_marker',
        ),
        Node(
            package='calibration_tool',
            executable='find_table_location',
        ),
        Node(
            package='my_tf_tools',
            executable='robot_hand_loc',
        )
    ])