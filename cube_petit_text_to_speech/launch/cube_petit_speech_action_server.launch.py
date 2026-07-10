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
"""Launch file for speech_action_server only (for standalone testing)."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace


def generate_launch_description() -> LaunchDescription:
    """Generate launch description."""
    # Default is empty (= no namespace push). Set robot_namespace:=cube_petit_orange
    # explicitly only for standalone testing.
    robot_namespace_arg = DeclareLaunchArgument(
        'robot_namespace',
        default_value='',
        description='Namespace of the robot unit (e.g. cube_petit_orange). Empty = no push.')

    return LaunchDescription([
        robot_namespace_arg,
        GroupAction([
            PushRosNamespace(LaunchConfiguration('robot_namespace')),
            Node(
                package='cube_petit_text_to_speech',
                executable='speech_action_server',
                name='speech_action_server',
                output='screen',
            ),
        ]),
    ])
