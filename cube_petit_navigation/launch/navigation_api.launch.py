#!/usr/bin/env python

# Copyright (c) 2025 SoftBank Corp.
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

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    places_file = PathJoinSubstitution([
        FindPackageShare('cube_petit_navigation'),
        'config',
        'places.yaml',
    ])

    return LaunchDescription([
        PushRosNamespace('cube_petit/navigation'),
        Node(
            package='cube_petit_navigation',
            executable='navigation_api_node',
            name='navigation_api_node',
            output='screen',
            parameters=[{
                'places_config_file': places_file,
            }],
        )
    ])
