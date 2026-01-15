#!/usr/bin/env python

# Copyright (c) 2026 SoftBank Corp.
# 
# <<licensetext>>


import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = 'cube_petit_bringup'

    default_param_file = os.path.join(
        FindPackageShare(pkg).find(pkg),
        'config',
        'realsense_bringup.yaml'
    )

    param_file = LaunchConfiguration('param_file')

    realsense_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='camera',
        namespace='camera',
        output='screen',
        parameters=[param_file],
        emulate_tty=True,
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'param_file',
            default_value=default_param_file,
            description='Path to realsense bringup yaml file'
        ),
        realsense_node,
    ])
