#!/usr/bin/env python
# -*- coding:utf-8 -*-

# Copyright (c) 2026 SoftBank Corp.
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
"""Launch the per-robot zenoh fleet connector.

The node is namespaced to `robot_namespace` so its relative topic names
(`navigation/goal`, `navigation/status`, `speech_action_server`, ...) resolve
exactly like navigation_api_node's / startup_announcer's own topics do on
this robot -- no manual prefixing needed.

`zenoh_router_endpoint` defaults to the router assumed to be running on
"orange" (tcp/cube-petit-orange.local:7447). Override it explicitly when
launching on another individual or once the real fleet-wide router address
is known.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Generate launch descriptions.

    Returns:
        Launch descriptions
    """
    args = [
        DeclareLaunchArgument(
            'robot_namespace',
            default_value='cube_petit_orange',
            description='Robot namespace. Also used to build the robots/<robot_namespace>/... zenoh keys '
            'unless robot_name is set explicitly.',
        ),
        DeclareLaunchArgument(
            'robot_name',
            default_value='',
            description='Override for the zenoh robot name. Empty means "use robot_namespace".',
        ),
        DeclareLaunchArgument(
            'zenoh_router_endpoint',
            default_value='tcp/cube-petit-orange.local:7447',
            description='zenoh router endpoint to connect to (router assumed to run on orange for now).',
        ),
        DeclareLaunchArgument(
            'zenoh_mode',
            default_value='client',
            description="zenoh session mode: 'client' (connect out to the router) or 'peer'.",
        ),
        DeclareLaunchArgument(
            'map_name',
            default_value='',
            description='Fixed map name to report. Empty triggers a best-effort auto-lookup from '
            "map_server's yaml_filename parameter, falling back to 'unknown'.",
        ),
        DeclareLaunchArgument(
            'initialpose_topic',
            default_value='navigation/initialpose',
            description="Topic the localize command publishes to (emcl2's AMCL-compatible initialpose input). "
            'NEEDS REAL-ROBOT VERIFICATION: confirm the fully qualified name with '
            "`ros2 topic list | grep initialpose` (see the PR's real-robot check steps).",
        ),
        DeclareLaunchArgument(
            'battery_level',
            default_value='1.0',
            description='TODO(battery): dummy fixed battery level (0-1) until real battery telemetry exists.',
        ),
        DeclareLaunchArgument(
            'state_publish_period_sec',
            default_value='1.0',
            description='Period [s] for the pose/battery/map_name zenoh state publish.',
        ),
    ]

    node = Node(
        package='cube_petit_fleet_bridge',
        executable='zenoh_connector',
        name='zenoh_connector',
        namespace=LaunchConfiguration('robot_namespace'),
        output='screen',
        # The connector dies if the router (on orange) is not up yet -- e.g. when
        # this unit boots faster than orange. Respawn so it reconnects on its own.
        respawn=True,
        respawn_delay=5.0,
        parameters=[{
            'robot_name': LaunchConfiguration('robot_name'),
            'zenoh_endpoint': LaunchConfiguration('zenoh_router_endpoint'),
            'zenoh_mode': LaunchConfiguration('zenoh_mode'),
            'map_name': LaunchConfiguration('map_name'),
            'initialpose_topic': LaunchConfiguration('initialpose_topic'),
            'battery_level': LaunchConfiguration('battery_level'),
            'state_publish_period_sec': LaunchConfiguration('state_publish_period_sec'),
        }],
    )

    return LaunchDescription(args + [node])
