#!/usr/bin/env python
# -*- coding:utf-8 -*-

# Copyright (c) 2024 SoftBank Corp.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Launch file."""
# import os

# from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
# from launch.actions import GroupAction
# from launch.actions import DeclareLaunchArgument
# from launch.actions import IncludeLaunchDescription
# from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Generate launch description."""

    # teleop_twist_joy_dir = get_package_share_directory('teleop_twist_joy')
    # cube_teleop_dir = get_package_share_directory('cube_petit_bringup')
    # joy_dev = '/dev/input/js0'
    # config_filepath = os.path.join(
    #     cube_teleop_dir, 'config', 'ps4.config.yaml'
    # )
    # teleop_include = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(os.path.join(teleop_twist_joy_dir, 'launch', 'teleop-launch.py')),
    #     launch_arguments={
    #         'joy_dev': joy_dev,
    #         'config_filepath': config_filepath
    #     }.items()
    # )

    return LaunchDescription([
        Node(
            package='cube_petit_text_to_speech',
            executable='cube_petit_text_to_jtalk.py',
            name='text_to_jtalk'
        ),
        Node(
            package='sbgisen_speech',
            executable='speech_action_server',
            name='speech_action_server'
        ),
        # Node(
        #     package='depthai_hand_tracker',
        #     executable='depthai_hand_tracker_ros.py',
        #     name='depthai_hand_tracker',
        #     output='screen'
        # ),
        # teleop_include
    ])
