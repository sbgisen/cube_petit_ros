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
#

import pathlib

import launch_ros.actions
import xacro
from launch import LaunchDescription
from launch.actions import EmitEvent
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Generate launch descriptions.

    Returns:
        Launch descriptions
    """
    pkg_share = FindPackageShare('cube_petit_description').find('cube_petit_description')
    xacro_file = pathlib.Path(pkg_share) / 'xacro/cube_petit.xacro'
    doc = xacro.process_file(xacro_file)
    robot_description = doc.toprettyxml(indent='  ')
    params = {'robot_description': robot_description}
    rsp = launch_ros.actions.Node(package='robot_state_publisher',
                                  executable='robot_state_publisher',
                                  output='both',
                                  parameters=[params])
    jsp = launch_ros.actions.Node(package='joint_state_publisher',
                                  executable='joint_state_publisher',
                                  output='both')
    rviz = launch_ros.actions.Node(package='rviz2',
                                   executable='rviz2',
                                   output='both',
                                   arguments=['-d', str(pathlib.Path(pkg_share) / 'rviz/urdf.rviz')])

    return LaunchDescription([
        rsp,
        jsp,
        rviz,
        RegisterEventHandler(event_handler=OnProcessExit(target_action=rviz,
                                                         on_exit=[EmitEvent(event=Shutdown())]))
    ])
