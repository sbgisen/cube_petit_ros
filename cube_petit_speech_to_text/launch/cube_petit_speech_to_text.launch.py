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
from launch_ros.actions import Node

from launch import LaunchDescription

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
# from launch.actions import GroupAction
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description() -> LaunchDescription:
    """Generate launch description."""
    return LaunchDescription([
        Node(
            package='cube_petit_speech_to_text',
            executable='cube_petit_speech_to_text',
            name='speech_to_text'
        ),
    ])
