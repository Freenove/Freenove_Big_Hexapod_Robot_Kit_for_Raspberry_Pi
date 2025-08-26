from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='windows_command',
            executable='main_window',
            name='main_window_node',
            output='screen'
        ),
    ])