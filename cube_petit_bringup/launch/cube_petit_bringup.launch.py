#!/usr/bin/env python3
# -*- coding:utf-8 -*-

# Copyright (c) 2024 SoftBank Corp.
#
# Licensed under the Apache License, Version 2.0 (the 'License');
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an 'AS IS' BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#

import pathlib
import socket

import xacro
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace



def generate_launch_description() -> LaunchDescription:
    """Generate launch descriptions.

    Returns:
        Launch descriptions
    """
    args = []
    args.append(DeclareLaunchArgument(
        'robot',
        default_value='cube_petit'))
    hostname = socket.gethostname()
    namespace = hostname.replace('-', '_')
    args.append(DeclareLaunchArgument('cube_petit_host_name', default_value=namespace))

    description_pkg = FindPackageShare('cube_petit_description').find('cube_petit_description')
    xacro_file = pathlib.Path(description_pkg) / 'xacro/cube_petit.xacro'
    doc = xacro.process_file(xacro_file, mappings={'use_sim': 'false'})
    robot_description = {"robot_description": doc.toprettyxml(indent='  ')}

    bringup_pkg = pathlib.Path(FindPackageShare('cube_petit_bringup').find('cube_petit_bringup'))
    face_animation_pkg = pathlib.Path(FindPackageShare(
            'cube_petit_facial_animation').find('cube_petit_facial_animation'))
    # TODO: joint_state_publisher?
    hardware_pkg = pathlib.Path(FindPackageShare('cube_petit_hardware_interface').find('cube_petit_hardware_interface'))
    speech_to_text_pkg = pathlib.Path(FindPackageShare('cube_petit_speech_to_text').find('cube_petit_speech_to_text'))
    text_to_speech_pkg = pathlib.Path(FindPackageShare('cube_petit_text_to_speech').find('cube_petit_text_to_speech'))


    bringup = GroupAction([
        PushRosNamespace(LaunchConfiguration('cube_petit_host_name')),
        Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description]
        ),
        IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(bringup_pkg / 'launch/teleop.launch.py'))),
        IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(text_to_speech_pkg / 'launch/cube_petit_text_to_jtalk.launch.py'))),
        IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(face_animation_pkg / 'launch/cube_petit_facial_animation.launch.py'))),
        IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(speech_to_text_pkg / 'launch/cube_petit_hotword_detector.launch.py'))),
        IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(speech_to_text_pkg / 'launch/cube_petit_speech_to_text.launch.py'))),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(hardware_pkg
                                                                      / 'launch/cube_petit_control.launch.py')),
                                                                      launch_arguments={}.items())
    ])


    return LaunchDescription(args + [
        bringup
    ])
