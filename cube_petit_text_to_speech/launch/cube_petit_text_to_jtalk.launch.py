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
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace


def generate_launch_description() -> LaunchDescription:
    """Generate launch description."""
    bringup_dir = get_package_share_directory('cube_petit_text_to_speech')
    controller_talk_yaml = os.path.join(bringup_dir, 'config', 'controller_talk.yaml')

    # Temporary standalone-test support: default is empty (= no namespace push,
    # no effect when included from bringup). Set robot_namespace:=cube_petit_orange
    # explicitly only for standalone testing.
    robot_namespace_arg = DeclareLaunchArgument(
        'robot_namespace',
        default_value='',
        description='Namespace of the robot unit (e.g. cube_petit_orange). Empty = no push.')

    # Per-robot voice, so a listener can tell orange/pink/violet apart during the
    # ROSConJP conversation demo (2026-08-04). Defaults reproduce the pre-existing
    # voice exactly. See cube_petit_text_to_speech/utils/jtalk.py (VOICE_PRESETS,
    # resolve_voice_params) for how preset + the two override params combine, and
    # cube_petit_text_to_speech/README.md for the tuning method.
    voice_preset_arg = DeclareLaunchArgument('voice_preset',
                                             default_value='default',
                                             description="Per-robot voice preset name: 'default' (orange, unchanged), "
                                             "'pink' (a bit higher & slower, soft), 'violet' (higher & a bit faster, "
                                             "playful). Unknown names fall back to 'default'.")
    voice_semitone_shift_arg = DeclareLaunchArgument(
        'voice_semitone_shift',
        default_value='0.0',
        description='Extra half-tone pitch shift added on top of voice_preset '
        "(open_jtalk -fm; e.g. '1.0' = a bit higher). 0.0 = no adjustment.")
    voice_speed_scale_arg = DeclareLaunchArgument(
        'voice_speed_scale',
        default_value='1.0',
        description='Extra speed multiplier applied on top of voice_preset and of '
        "each utterance's own speed (e.g. '1.05' = 5% faster). 1.0 = no adjustment.")
    voice_name_arg = DeclareLaunchArgument('voice_name',
                                           default_value='mei',
                                           description='htsvoice model folder name under MMDAgent_Example-1.6/Voice/. '
                                           'Only "mei" is bundled today; switching requires installing another '
                                           '5-emotion htsvoice set with the same file-naming convention.')

    # teleop_twist_joy_dir = get_package_share_directory('teleop_twist_joy')
    # cube_teleop_dir = get_package_share_directory('cube_petit_bringup')
    # joy_dev = '/dev/input/js0'
    # config_filepath = os.path.join(
    #     cube_teleop_dir, 'config', 'ps4.config.yaml'
    # )
    # teleop_include = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(os.path.join(teleop_twist_joy_dir, 'launch', 'teleop-launch.py')),
    #     launch_arguments={
    #         'joy_dev': joy_dev,
    #         'config_filepath': config_filepath
    #     }.items()
    # )
    # depthai_hand_tracker_dir = get_package_share_directory('depthai_hand_tracker')
    # depthai_include = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(depthai_hand_tracker_dir, 'launch', 'depthai_hand_tracker.launch.py')),
    # )

    return LaunchDescription([
        robot_namespace_arg,
        voice_preset_arg,
        voice_semitone_shift_arg,
        voice_speed_scale_arg,
        voice_name_arg,
        GroupAction([
            PushRosNamespace(LaunchConfiguration('robot_namespace')),
            Node(
                package='cube_petit_text_to_speech',
                executable='cube_petit_text_to_jtalk',
                name='text_to_jtalk',
                parameters=[{
                    'controller_talk_config': controller_talk_yaml
                }],
            ),
            Node(package='cube_petit_text_to_speech',
                 executable='speech_action_server',
                 name='speech_action_server',
                 parameters=[{
                     'voice_preset': LaunchConfiguration('voice_preset'),
                     'voice_semitone_shift': LaunchConfiguration('voice_semitone_shift'),
                     'voice_speed_scale': LaunchConfiguration('voice_speed_scale'),
                     'voice_name': LaunchConfiguration('voice_name'),
                 }]),
        ]),

        # teleop_include
        # depthai_include,
    ])
