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
#

import pathlib
import socket

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.launch_context import LaunchContext
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace
from launch_ros.substitutions import FindPackageShare
import xacro

# from launch.actions import RegisterEventHandler
# from launch.event_handlers import OnProcessStart


def launch_in_order(context: LaunchContext, *args, **kwargs) -> list:
    ns = LaunchConfiguration('cube_petit_host_name').perform(context)

    hardware_pkg = pathlib.Path(
        FindPackageShare('cube_petit_hardware_interface').find('cube_petit_hardware_interface'))
    bringup_pkg = pathlib.Path(FindPackageShare('cube_petit_bringup').find('cube_petit_bringup'))
    speech_to_text_pkg = pathlib.Path(FindPackageShare('cube_petit_speech_to_text').find('cube_petit_speech_to_text'))
    text_to_speech_pkg = pathlib.Path(FindPackageShare('cube_petit_text_to_speech').find('cube_petit_text_to_speech'))
    face_animation_pkg = pathlib.Path(
        FindPackageShare('cube_petit_facial_animation').find('cube_petit_facial_animation'))
    description_pkg = FindPackageShare('cube_petit_description').find('cube_petit_description')
    xacro_file = pathlib.Path(description_pkg) / 'xacro/cube_petit.xacro'
    doc = xacro.process_file(xacro_file, mappings={'use_sim': 'false'})
    robot_description = {'robot_description': doc.toprettyxml(indent='  ')}

    robot_state_publisher = GroupAction([
        PushRosNamespace(ns),
        Node(package='robot_state_publisher', executable='robot_state_publisher', parameters=[robot_description])
    ])

    control_node = GroupAction([
        PushRosNamespace(ns),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            str(hardware_pkg / 'launch/cube_petit_control.launch.py')),
                                 launch_arguments={}.items())
    ])

    bringups = GroupAction([
        PushRosNamespace(ns),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            str(face_animation_pkg / 'launch/cube_petit_facial_animation.launch.py')),
                                 launch_arguments={
                                     'color': LaunchConfiguration('face_color'),
                                 }.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(bringup_pkg / 'launch/lidar.launch.py'))),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(bringup_pkg / 'launch/depth.launch.py'))),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(bringup_pkg / 'launch/teleop.launch.py'))),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(text_to_speech_pkg / 'launch/cube_petit_text_to_jtalk.launch.py'))),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(speech_to_text_pkg / 'launch/cube_petit_speech_to_text.launch.py'))),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(speech_to_text_pkg / 'launch/cube_petit_hotword_detector.launch.py'))),
        Node(
            package='cube_petit_bringup',
            executable='startup_announcer',
            name='startup_announcer',
            output='screen',
            parameters=[{
                'text': '起動しました。',
                'required_nodes': [
                    f'/{ns}/speech_action_server',
                    f'/{ns}/text_to_jtalk',
                    f'/{ns}/ldlidar_publisher_ld06',
                    f'/{ns}/controller_manager',
                ],
                'required_scan_topic': f'/{ns}/scan',
                'controller_manager_service': f'/{ns}/controller_manager',
                'required_controller_types': [
                    'joint_state_broadcaster/JointStateBroadcaster',
                    'diff_drive_controller/DiffDriveController',
                ],
                'node_check_timeout_sec': 15.0,
                'topic_check_timeout_sec': 5.0,
                'controller_check_timeout_sec': 30.0,
                'controller_grace_sec': 10.0,
                'wait_sec': 2.0,
            }],
        ),
    ])

    return [robot_state_publisher, bringups, control_node]


def generate_launch_description() -> LaunchDescription:
    """Generate launch descriptions.

    Returns:
        Launch descriptions
    """
    args = []
    hostname = socket.gethostname()
    namespace = hostname.replace('-', '_')
    args.append(DeclareLaunchArgument('cube_petit_host_name', default_value=namespace))
    args.append(
        DeclareLaunchArgument('face_color',
                              default_value='orange',
                              description='Color for the faceBox (pink, orange, blue, green, yellow, purple)'))

    ordered_sequence = OpaqueFunction(function=launch_in_order)

    return LaunchDescription(args + [ordered_sequence])
