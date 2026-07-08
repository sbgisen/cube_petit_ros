#!/usr/bin/env python

# Copyright (c) 2025 SoftBank Corp.
# 
# <<licensetext>>


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    robot_arg = DeclareLaunchArgument('robot', default_value='cube_petit_orange', description='Robot namespace.')
    output_robot_arg = DeclareLaunchArgument(
        'output_robot', default_value='cube_petit_pink', description='Relay target robot namespace.')

    return LaunchDescription([
        robot_arg,
        output_robot_arg,
        Node(
            package='topic_tools',
            executable='relay',
            name='cmd_vel_relay',
            parameters=[
                {
                    'input_topic': ['/', LaunchConfiguration('robot'), '/diff_drive_controller/cmd_vel'],
                    'output_topic': ['/', LaunchConfiguration('output_robot'), '/diff_drive_controller/cmd_vel']
                }
            ]
        )
    ])
