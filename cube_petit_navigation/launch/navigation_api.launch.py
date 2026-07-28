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
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
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
        DeclareLaunchArgument('robot', default_value='cube_petit', description='Robot namespace.'),
        # navigation_api_node自身のtopic/service名はコード側で既に'navigation/'を
        # 前置している(navigation/goal, navigation/status, navigation/save_place等)。
        # ここでさらに'/navigation'をpushすると実際の購読先が
        # '<robot>/navigation/navigation/goal'のように二重になり、zenoh_connectorが
        # publishする'<robot>/navigation/goal'(navigation1階層)と一致せず
        # ゴールが一切届かない不具合になっていた(2026-07-28、実機で発見。
        # move_to_poseが永遠に"in flight"のまま完了しなかった根本原因の一つ)。
        # The node's own topic/service names already carry the 'navigation/' prefix
        # in code (navigation/goal, navigation/status, navigation/save_place, ...).
        # Pushing an extra '/navigation' namespace here doubled it, so the actual
        # subscribed topic became '<robot>/navigation/navigation/goal', not matching
        # zenoh_connector's '<robot>/navigation/goal' publisher (found on real
        # hardware, 2026-07-28 -- root cause of move_to_pose goals never being
        # received at all).
        PushRosNamespace([LaunchConfiguration('robot')]),
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
