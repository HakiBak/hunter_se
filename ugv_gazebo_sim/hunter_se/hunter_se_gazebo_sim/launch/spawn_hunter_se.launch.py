#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    namespace = LaunchConfiguration("namespace", default="hunter_se")
    x_pose = LaunchConfiguration("x_pose", default="0.0")
    y_pose = LaunchConfiguration("y_pose", default="0.0")
    yaw_pose = LaunchConfiguration("yaw_pose", default="0.0")

    declare_namespace_arg = DeclareLaunchArgument(
        "namespace", default_value=namespace, description="Specify robot namespace"
    )
    declare_x_pose_arg = DeclareLaunchArgument(
        "x_pose", default_value=x_pose, description="Specify robot x position"
    )
    declare_y_pose_arg = DeclareLaunchArgument(
        "y_pose", default_value=y_pose, description="Specify robot y position"
    )
    declare_yaw_pose_arg = DeclareLaunchArgument(
        "yaw_pose", default_value=yaw_pose, description="Specify robot yaw angle"
    )

    start_gazebo_ros_spawner_cmd = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic", "hunter_se/robot_description",
            "-name",  "hunter_se",
            "-x", x_pose,
            "-y", y_pose,
            "-z", "0.31",
            "-Y", yaw_pose,
        ],
        output="screen",
    )

    ld = LaunchDescription()
    ld.add_action(declare_namespace_arg)
    ld.add_action(declare_x_pose_arg)
    ld.add_action(declare_y_pose_arg)
    ld.add_action(declare_yaw_pose_arg)
    ld.add_action(start_gazebo_ros_spawner_cmd)

    return ld
