import os
import launch
import launch_ros.actions
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    bringup_dir = get_package_share_directory('g1_description')
    urdf_dir = os.path.join(bringup_dir, 'urdf')
    urdf_file = os.path.join(urdf_dir, 'g1_body29_hand14.urdf')

    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    params = {'robot_description': robot_desc}

    #Decide to use Rviz or not
    use_rviz = launch.substitutions.LaunchConfiguration('use_rviz', default=False)

    return launch.LaunchDescription([
        # Load and publish the robot description
        launch_ros.actions.Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[params],
        ),
        
        # Start RViz for visualization
        launch_ros.actions.Node(
            condition=IfCondition(use_rviz),
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            parameters=[{'use_sim_time': False}]
        ),
    ])