"""
Launch file for Hunter SE RL training.
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    hunter_se_gazebo_dir = get_package_share_directory('hunter_se_gazebo_sim')

    return LaunchDescription([
        DeclareLaunchArgument('world_name', default_value='obstacles_easy.world'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(hunter_se_gazebo_dir, 'launch', 'hunter_se_empty_world.launch.py')),
            launch_arguments={'world_name': LaunchConfiguration('world_name')}.items()),
    ])