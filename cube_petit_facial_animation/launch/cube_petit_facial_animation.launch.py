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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace


def generate_launch_description() -> LaunchDescription:
    """Face animation."""
    return LaunchDescription([
        DeclareLaunchArgument('color',
                              default_value='blue',
                              description='Color for the faceBox (pink, orange, blue, green, yellow, purple)'),
        # 【一時的な単体テスト用】default_valueは空文字(=名前空間なし。bringup経由の起動には無影響)。
        # 単体テスト時だけ robot:=cube_petit_orange のように明示的に指定する。
        DeclareLaunchArgument('robot', default_value='', description='Robot namespace (単体テスト用、空なら無効).'),
        PushRosNamespace(LaunchConfiguration('robot')),
        Node(
            package='cube_petit_facial_animation',
            executable='animation.py',
            name='facial_animation',
            output='screen',
            parameters=[{
                'color': LaunchConfiguration('color')
            }],
        ),
        Node(
            package='cube_petit_facial_animation',
            executable='expression_operator.py',
            name='expression_operator',
            output='screen',
        ),
        Node(package='rosbridge_server', executable='rosbridge_websocket', name='rosbridge_websocket', output='screen')
    ])


if __name__ == '__main__':
    generate_launch_description()
