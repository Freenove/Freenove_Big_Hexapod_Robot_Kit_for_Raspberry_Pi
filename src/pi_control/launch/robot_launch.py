from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    realsense_launch_path = os.path.join(
        get_package_share_directory('realsense2_camera'),
        'launch',
        'rs_launch.py'
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(realsense_launch_path),
            launch_arguments={
                'enable_depth': 'true',
                'align_depth.enable': 'true',

                'enable_color': 'false',
                'enable_infra1': 'false',
                'enable_infra2': 'false',
                'enable_gyro': 'false',
                'enable_accel': 'false',
                'enable_pointcloud': 'false',
            }.items(),
        ),

        Node(
            package='pi_control',
            executable='servo_node',
            name='servo_node',
            output='screen'
        )
    ])