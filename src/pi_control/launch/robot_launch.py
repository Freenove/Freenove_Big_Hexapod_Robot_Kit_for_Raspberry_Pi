from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    realsense_launch_path = os.path.join(
        get_package_share_directory('realsense2_camera'),
        'launch', 'rs_launch.py'
    )

    realsense_launch_arguments = {
        'depth_module.depth_profile': '480x270x6',
        'depth_module.infra_profile': '480x270x6',
        'rgb_camera.color_profile': '424x240x6',
        'enable_color': 'true',
        'align_depth.enable': 'true',
        'enable_pointcloud': 'true'

    }

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(realsense_launch_path),
            launch_arguments=realsense_launch_arguments.items()
        ),

        Node(
            package='pi_control',
            executable='servo_node',
            name='servo_node',
            output='screen'
        )
    ])