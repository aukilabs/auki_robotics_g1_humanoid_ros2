from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Sensors
    mid360_file = os.path.join(FindPackageShare('g1_bringup').find('g1_bringup'), 'launch', 'sensors', 'mid360.launch.py')
    rs_file = os.path.join(FindPackageShare('realsense2_camera').find('realsense2_camera'), 'launch', 'rs_launch.py')
    rs_config = os.path.join(FindPackageShare('g1_bringup').find('g1_bringup'), 'config', 'sensors', 'realsense.yaml')

    # Description
    g1_description_file = os.path.join(FindPackageShare('g1_description').find('g1_description'), 'launch', 'g1_description.launch.py')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(mid360_file)
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rs_file),
            launch_arguments={
                'camera_name': 'd435',
                'camera_namespace': 'rs_camera',
                'config_file': rs_config
            }.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(g1_description_file)
        ),
        Node(
            package='g1_control_py',
            executable='lowstate_translator',
        ),
        Node(
            package='g1_control_py',
            executable='cmd_vel_translator',
        ),
    ])