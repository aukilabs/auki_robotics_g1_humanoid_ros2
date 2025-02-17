from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
import os

def generate_launch_description():
    g1_description_file = os.path.join(FindPackageShare('g1_description').find('g1_description'), 'launch', 'g1_description.launch.py')

    return LaunchDescription([
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