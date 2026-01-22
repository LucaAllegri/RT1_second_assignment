from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('bme_gazebo_sensors'),
                'launch',
                'spawn_robot.launch.py'
            )
        )
    )
    
    #UI NODE
    input_controller_node = Node(
        package='robot_controller',
        executable='ui',
        name='ui_node',
        output='screen',
        prefix='xterm -title "USER INTERFACE" -e'
    )
    
    #DISTANCE NODE
    distance_controller_node = Node(
        package='robot_controller',
        executable='distance',
        name='distance_node',
        output='screen', 
        prefix='xterm -title "LOG" -e',
        parameters=[{
            'threshold': 0.8
        }]
    )

    #STATUS NODE
    status_node = Node(
        package='robot_controller',
        executable='status',
        name='status_node',
        output='screen',
        prefix='xterm -title "ROBOT STATUS" -e',
    )

    return LaunchDescription([
        gazebo_launch,
        input_controller_node,
        distance_controller_node,
        status_node,
    ])
