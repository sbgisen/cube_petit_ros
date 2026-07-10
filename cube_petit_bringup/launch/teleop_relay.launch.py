#!/usr/bin/env python3
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
#

import socket

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    hostname = socket.gethostname()
    namespace = hostname.replace('-', '_')

    robot_namespace_arg = DeclareLaunchArgument(
        'robot_namespace',
        default_value=namespace,
        description='Namespace of the robot unit to relay cmd_vel from (e.g. cube_petit_orange).')
    output_robot_namespace_arg = DeclareLaunchArgument(
        'output_robot_namespace',
        default_value='cube_petit_pink',
        description='Namespace of the relay target robot unit to relay cmd_vel to.')

    return LaunchDescription([
        robot_namespace_arg, output_robot_namespace_arg,
        Node(
            package='topic_tools',
            executable='relay',
            name='cmd_vel_relay',
            parameters=[{
                'input_topic': ['/', LaunchConfiguration('robot_namespace'), '/diff_drive_controller/cmd_vel'],
                'output_topic': ['/',
                                 LaunchConfiguration('output_robot_namespace'), '/diff_drive_controller/cmd_vel']
            }])
    ])
