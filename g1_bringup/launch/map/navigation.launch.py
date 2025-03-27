from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Navigation
    # nav_file = os.path.join(FindPackageShare('nav2_bringup').find('nav2_bringup'), 'launch', 'bringup_launch.py')
    nav_file = os.path.join(FindPackageShare('nav2_bringup').find('nav2_bringup'), 'launch', 'navigation_launch.py')
    # nav_params_file = os.path.join(FindPackageShare('g1_bringup').find('g1_bringup'), 'config', 'navigation', 'humble_navigation.yaml') # stvl
    nav_params_file = os.path.join(FindPackageShare('g1_bringup').find('g1_bringup'), 'config', 'navigation', 'stvl_navigation.yaml') # stvl
    # nav_params_file = os.path.join(FindPackageShare('g1_bringup').find('g1_bringup'), 'config', 'navigation', 'octomap_navigation.yaml') # octomap
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav_file),
            launch_arguments={
                'slam': 'False',
                'use_sim_time': 'False',
                'params_file': nav_params_file
            }.items()
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            output="screen" ,
            arguments=["0", "0", "0", "0", "0", "0", "map", "odom"]
        )
    ])