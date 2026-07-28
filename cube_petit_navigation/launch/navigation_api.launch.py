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
        # navigation_api_nodeは'<robot>/navigation'名前空間で動く必要がある:
        # (1) 自身のtopic/service名(goal, status, save_place等、'navigation/'接頭辞は
        #     コード側から削除済み)がここに相対解決されてzenoh_connectorの
        #     '<robot>/navigation/goal'と一致する。
        # (2) CubePetitNavigationCommanderのActionClientが相対名'navigate_to_pose'を
        #     使っており、nav2のbt_navigator(navigation.launch.py側でこの名前空間に
        #     pushされている)が公開する'<robot>/navigation/navigate_to_pose'と
        #     一致する必要がある。
        # 以前(2026-07-28)、(1)のtopic名重複だけを見て namespace を'<robot>'のみに
        # 変更したが、それは(2)を壊し、ActionClient.wait_for_server()が永遠に
        # ブロックしてnavigation_api_nodeがrclpy.spin()に到達できず、結局goalも
        # 一切処理されなくなっていた(実機で発見)。正しい修正はnamespaceを
        # '<robot>/navigation'に戻し、ノード側のtopic名から冗長な'navigation/'
        # 接頭辞を外すことだった。
        # navigation_api_node must run under the '<robot>/navigation' namespace:
        # (1) its own topic/service names (goal, status, save_place, ... -- the
        #     'navigation/' prefix was removed from the code) resolve relative to
        #     it, matching zenoh_connector's '<robot>/navigation/goal'.
        # (2) CubePetitNavigationCommander's ActionClient uses the relative name
        #     'navigate_to_pose', which must match nav2's bt_navigator (pushed into
        #     this same namespace by navigation.launch.py) at
        #     '<robot>/navigation/navigate_to_pose'.
        # Previously (2026-07-28) only (1)'s doubled topic name was noticed and the
        # namespace was changed to just '<robot>', which broke (2) instead:
        # ActionClient.wait_for_server() blocked forever, so navigation_api_node
        # never reached rclpy.spin() and no goal was ever processed either (found on
        # real hardware). The correct fix is to keep this namespace and instead drop
        # the redundant 'navigation/' prefix from the node's own topic names.
        PushRosNamespace([LaunchConfiguration('robot'), '/navigation']),
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
