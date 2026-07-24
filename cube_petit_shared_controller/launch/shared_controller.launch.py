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
"""Launch either the hub or the receiver half of the shared PS4 controller relay.

Usage::

    # On the hub individual (controller physically paired here, e.g. orange). This node
    # does NOT start joy_node itself -- also launch cube_petit_bringup/launch/teleop.launch.py
    # (or the full cube_petit_bringup.launch.py) so /joy exists. See this package's README.
    ros2 launch cube_petit_shared_controller shared_controller.launch.py role:=hub

    # On every controllable individual, including the hub one (e.g. orange AND pink). Do NOT
    # also launch teleop.launch.py here: this individual has no controller physically attached,
    # and running its joy_node/teleop_twist_joy_node would be pointless (no /dev/input/js0).
    ros2 launch cube_petit_shared_controller shared_controller.launch.py role:=receiver
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.actions import OpaqueFunction
from launch.launch_context import LaunchContext
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace


def _launch_setup(context: LaunchContext, *args, **kwargs) -> list:
    role = LaunchConfiguration('role').perform(context)
    robot_namespace = LaunchConfiguration('robot_namespace')

    if role == 'hub':
        robot_names = [
            name.strip() for name in LaunchConfiguration('robot_names').perform(context).split(',') if name.strip()
        ]
        switch_button = int(LaunchConfiguration('switch_button').perform(context))
        joy_topic = LaunchConfiguration('joy_topic').perform(context)
        local_cmd_vel_topic = LaunchConfiguration('local_cmd_vel_topic').perform(context)
        ps4_config_filepath = os.path.join(get_package_share_directory('cube_petit_bringup'), 'config',
                                           'ps4.config.yaml')

        return [
            GroupAction([
                PushRosNamespace(robot_namespace),
                # Dedicated teleop_twist_joy_node instance, reusing the same PS4 mapping as
                # teleop.launch.py's own instance, but publishing to a *different* topic
                # (local_cmd_vel_topic) so this individual's controller_receiver_node (also
                # running here, see role:=receiver above) never feeds this relay's own
                # subscription back into itself. Does not start its own joy_node: it reads
                # the /joy topic already published by teleop.launch.py.
                Node(
                    package='teleop_twist_joy',
                    executable='teleop_node',
                    name='shared_controller_teleop_twist_joy_node',
                    parameters=[ps4_config_filepath, {
                        'publish_stamped_twist': False,
                    }],
                    remappings=[('joy', joy_topic), ('cmd_vel', local_cmd_vel_topic)],
                ),
                Node(
                    package='cube_petit_shared_controller',
                    executable='controller_hub_node',
                    name='controller_hub_node',
                    output='screen',
                    parameters=[{
                        'robot_names': robot_names,
                        'switch_button': switch_button,
                        'joy_topic': joy_topic,
                        'local_cmd_vel_topic': local_cmd_vel_topic,
                        'zenoh_endpoint': LaunchConfiguration('zenoh_router_endpoint'),
                        'zenoh_mode': LaunchConfiguration('zenoh_mode'),
                        'initial_robot_name': LaunchConfiguration('initial_robot_name'),
                    }],
                ),
            ]),
        ]

    if role == 'receiver':
        return [
            GroupAction([
                PushRosNamespace(robot_namespace),
                Node(
                    package='cube_petit_shared_controller',
                    executable='controller_receiver_node',
                    name='controller_receiver_node',
                    output='screen',
                    parameters=[{
                        'robot_name': LaunchConfiguration('robot_name'),
                        'output_cmd_vel_topic': LaunchConfiguration('output_cmd_vel_topic'),
                        'zenoh_endpoint': LaunchConfiguration('zenoh_router_endpoint'),
                        'zenoh_mode': LaunchConfiguration('zenoh_mode'),
                        'announcement_enabled': LaunchConfiguration('announcement_enabled'),
                        'announcement_text': LaunchConfiguration('announcement_text'),
                    }],
                ),
            ]),
        ]

    raise RuntimeError(f"Unknown role {role!r}: expected 'hub' or 'receiver'")


def generate_launch_description() -> LaunchDescription:
    """Generate launch descriptions.

    Returns:
        Launch descriptions
    """
    args = [
        DeclareLaunchArgument('role',
                              default_value='hub',
                              description="Which half to launch: 'hub' (controller is physically paired here) "
                              "or 'receiver' (this individual is remote-controllable, hub included)."),
        DeclareLaunchArgument('robot_namespace',
                              default_value='cube_petit_orange',
                              description='ROS namespace of this individual (matches cube_petit_bringup.launch.py'
                              "'s cube_petit_host_name, e.g. cube_petit_orange / cube_petit_pink)."),
        # ---- role:=hub only ----
        DeclareLaunchArgument('robot_names',
                              default_value='cube_petit_orange,cube_petit_pink',
                              description='Comma-separated candidate robot names to cycle through on '
                              'switch_button. Order matters (toggle order).'),
        DeclareLaunchArgument('switch_button',
                              default_value='2',
                              description='sensor_msgs/Joy buttons[] index that cycles the selected robot on '
                              'its rising edge (nominally Triangle on a PS4 pad). Buttons 0 (X, enable_button) '
                              'and 5 (L1, enable_turbo_button) are already used by ps4.config.yaml -- do not '
                              'reuse those. NEEDS REAL-ROBOT VERIFICATION: confirm the actual index with '
                              '`ros2 topic echo <joy_topic>` while pressing the intended button.'),
        DeclareLaunchArgument('joy_topic',
                              default_value='diff_drive_controller/joy',
                              description='Local /joy topic (relative to robot_namespace) published by '
                              "teleop.launch.py's joy_node. Must match teleop.launch.py's own namespacing."),
        DeclareLaunchArgument('local_cmd_vel_topic',
                              default_value='diff_drive_controller/shared_controller/local_cmd_vel',
                              description='Internal-only topic the dedicated teleop_twist_joy_node instance '
                              'publishes to. Deliberately distinct from diff_drive_controller/cmd_vel; see '
                              "this file's module docstring."),
        DeclareLaunchArgument('initial_robot_name',
                              default_value='',
                              description='Robot selected at startup, before any button press. Empty means '
                              'robot_names[0].'),
        # ---- role:=receiver only ----
        DeclareLaunchArgument('robot_name',
                              default_value='',
                              description="This individual's name, compared against controller/selected_robot. "
                              'Empty means: use the ROBOT_NAMESPACE env var if set, else robot_namespace.'),
        DeclareLaunchArgument('output_cmd_vel_topic',
                              default_value='diff_drive_controller/cmd_vel',
                              description='Real actuator topic (relative to robot_namespace) to publish '
                              'TwistStamped to while selected.'),
        DeclareLaunchArgument('announcement_enabled',
                              default_value='true',
                              description='Whether to speak an "it\'s me" announcement when this individual '
                              'becomes the selected robot.'),
        DeclareLaunchArgument('announcement_text',
                              default_value='自分だよ!',
                              description='Text spoken via speech_action_server when selected.'),
        # ---- shared ----
        DeclareLaunchArgument('zenoh_router_endpoint',
                              default_value='tcp/cube-petit-orange.local:7447',
                              description='zenoh router endpoint (router assumed to run on orange, same '
                              'default as cube_petit_fleet_bridge).'),
        DeclareLaunchArgument('zenoh_mode',
                              default_value='client',
                              description="zenoh session mode: 'client' (connect out to the router) or 'peer'."),
    ]

    return LaunchDescription(args + [OpaqueFunction(function=_launch_setup)])
