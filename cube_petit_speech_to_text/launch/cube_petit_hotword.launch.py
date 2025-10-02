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
"""Launch file."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    base_model = LaunchConfiguration('base_model')
    config_file_path = LaunchConfiguration('config_file_path')

    return LaunchDescription([
        DeclareLaunchArgument(
            'base_model',
            default_value='resnet50',
            description='Base model to use'
        ),
        DeclareLaunchArgument(
            'config_file_path',
            default_value=os.path.join(
                get_package_share_directory('cube_petit_speech_to_text'),
                'config',
                'efficientword_net.yaml'
            ),
            description='Path to the config file'
        ),
        SetEnvironmentVariable(
            'OMP_NUM_THREADS', '1'
        ),
        Node(
            package='cube_petit_speech_to_text',
            executable='cube_petit_hotword.py',
            name='hotword_detector',
            output='screen',
            parameters=[
                {'base_model': base_model},
                config_file_path
            ]
        )
    ])

if __name__ == '__main__':
    generate_launch_description()
