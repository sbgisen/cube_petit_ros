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

import xacro
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import EmitEvent
from launch.actions import IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Generate launch descriptions.

    Returns:
        Launch descriptions
    """
    args = []
    args.append(DeclareLaunchArgument(
        'robot',
        default_value='cube_petit'))

    description_pkg = FindPackageShare('cube_petit_description').find('cube_petit_description')
    xacro_file = pathlib.Path(description_pkg) / 'xacro/cube_petit.xacro'
    doc = xacro.process_file(xacro_file, mappings={'use_sim': 'false'})
    robot_description = {"robot_description": doc.toprettyxml(indent='  ')}

    bringup_pkg = pathlib.Path(FindPackageShare('cube_petit_bringup').find('cube_petit_bringup'))

    # TODO: joint_state_publisher?

    robot_state = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description]
    )

    general = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(bringup_pkg / 'launch/include/general_bringup.launch.py')),
        launch_arguments={
            'robot': LaunchConfiguration('robot')}.items())

    return LaunchDescription(args + [
        robot_state,
        RegisterEventHandler(event_handler=OnProcessExit(target_action=robot_state,
                                                         on_exit=[EmitEvent(event=Shutdown())])),
        general,
    ])
