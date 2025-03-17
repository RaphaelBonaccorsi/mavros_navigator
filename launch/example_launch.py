


import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # Specify the actions
    navigator_cmd = Node(
        package='mavros_navigator',
        executable='mavros_navigator.py',
        name='mavros_navigator',
        output='screen',
        parameters=[])

    path_planner_cmd = Node(
        package='mavros_navigator',
        executable='path_planner.py',
        name='path_planner',
        output='screen',
        parameters=[])

    

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options

    ld.add_action(navigator_cmd)
    ld.add_action(path_planner_cmd)

    return ld
