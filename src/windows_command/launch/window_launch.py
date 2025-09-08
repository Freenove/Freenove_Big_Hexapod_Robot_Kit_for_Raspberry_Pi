from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():

    start_gui_node = ExecuteProcess(
        cmd=['ros2', 'run', 'windows_command', 'hexapod_gui'],
        output='screen',
        emulate_tty=True  # Helps with terminal output formatting
    )


    return LaunchDescription([
        start_gui_node
    ])