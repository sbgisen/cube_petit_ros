#!/usr/bin/env python3
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
"""Launch conversation-only demo bringup.

Bundles facial animation, TTS (text_to_jtalk + speech_action_server), and the
zenoh fleet connector, with no hardware/motor control, lidar, depth camera,
teleop, or twist_mux. Intended for units that cannot drive (e.g. violet,
which lacks working DAMIAO motor control) but still need to run conversation
demos: face + speech, driven remotely over zenoh by another individual's
scenario/interaction stack.

Microphone-side nodes (speech_to_text / hotword_detector) are intentionally
excluded -- by design only orange's microphone picks up the human voice.
"""

import pathlib

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Generate the conversation-only demo launch description."""
    face_animation_pkg = pathlib.Path(
        FindPackageShare('cube_petit_facial_animation').find('cube_petit_facial_animation'))
    text_to_speech_pkg = pathlib.Path(FindPackageShare('cube_petit_text_to_speech').find('cube_petit_text_to_speech'))
    fleet_bridge_pkg = pathlib.Path(FindPackageShare('cube_petit_fleet_bridge').find('cube_petit_fleet_bridge'))

    args = [
        DeclareLaunchArgument('robot_namespace',
                              description='Namespace of the robot unit (e.g. cube_petit_violet). Required.'),
        DeclareLaunchArgument('face_color',
                              default_value='purple',
                              description='Color for the faceBox (pink, orange, blue, green, yellow, purple). '
                              'When passing a hex value on the CLI, quote it as a YAML string literal '
                              '(e.g. face_color:=\'"#8a2be2"\'), otherwise launch/YAML parses the leading '
                              "'#' as a comment and silently drops the rest of the argument."),
        DeclareLaunchArgument('zenoh_router_endpoint',
                              default_value='tcp/cube-petit-orange.local:7447',
                              description='zenoh router endpoint to connect to (router assumed to run on orange).'),
        DeclareLaunchArgument('zenoh_mode',
                              default_value='client',
                              description="zenoh session mode: 'client' (connect out to the router) or 'peer'."),
    ]

    # face + speech are namespaced by pushing robot_namespace onto the group and
    # relying on the included launch files' own empty-default robot_namespace
    # (same pattern as cube_petit_bringup.launch.py's `bringups` group).
    demo_group = GroupAction([
        PushRosNamespace(LaunchConfiguration('robot_namespace')),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            str(face_animation_pkg / 'launch/cube_petit_facial_animation.launch.py')),
                                 launch_arguments={
                                     'color': LaunchConfiguration('face_color'),
                                 }.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(text_to_speech_pkg / 'launch/cube_petit_text_to_jtalk.launch.py'))),
    ])

    # zenoh_connector applies its own namespace via the Node's `namespace=` field
    # (see zenoh_connector.launch.py), so it must stay outside the PushRosNamespace
    # group above -- nesting it there would double-push the namespace.
    fleet_bridge = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        str(fleet_bridge_pkg / 'launch/zenoh_connector.launch.py')),
                                            launch_arguments={
                                                'robot_namespace': LaunchConfiguration('robot_namespace'),
                                                'zenoh_router_endpoint': LaunchConfiguration('zenoh_router_endpoint'),
                                                'zenoh_mode': LaunchConfiguration('zenoh_mode'),
                                            }.items())

    return LaunchDescription(args + [demo_group, fleet_bridge])
