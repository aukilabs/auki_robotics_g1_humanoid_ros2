import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('g1_bringup'),  # Replace with your package name
        'config',
        'map',
        'octomap_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='octomap_server',
            executable='octomap_server_node',
            name='octomap_server',
            output='screen',
            parameters=[config],  # Load parameters from YAML
            remappings=[
                ('cloud_in', '/livox/lidar')
            ]
        )
    ])