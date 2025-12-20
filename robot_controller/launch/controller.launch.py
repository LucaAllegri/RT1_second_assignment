# launch_turtles.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    #RVIZ ENVIRONMNET
    turtlesim_node = Node(
        package='bme_gazebo_sensors',
        executable='spawn_robot',
        name='spawn_robot',
        output='screen'
    )

    #TURTLE SPAWN NODE
    turtle_spawn_node = Node(
        package='pck_assignment1',
        executable='turtle_spawn.py', 
        name='turtle_spawn',
        output='screen',
    )

    #UI NODE
    input_controller_node = Node(
        package='pck_assignment1',
        executable='ui', 
        name='ui_node',
        output='screen',
        prefix='xterm -e'
    )
    
    #DISTANCE NODE
    distance_controller_node = Node(
        package='pck_assignment1', 
        executable='distance',
        name='distance_node',
        output='screen', 
        prefix='xterm -e'
    )

    return LaunchDescription([
        turtlesim_node,
        turtle_spawn_node,
        input_controller_node,
        distance_controller_node
    ])

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

    input_controller_node = Node(
        package='robot_controller',
        executable='ui',
        name='ui_node'
    )
    
    #DISTANCE NODE
    distance_controller_node = Node(
        package='robot_controller',
        executable='distance',
        name='distance_node'
    )

    return LaunchDescription([
        gazebo_launch,
        input_controller_node,
        distance_controller_node,
    ])
