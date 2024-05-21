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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument


def generate_launch_description() -> LaunchDescription:
    """Generate launch descriptions.

    Returns:
        Launch descriptions
    """
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')
    robot_namespace_arg = DeclareLaunchArgument(
        'robot_namespace',
        default_value='cube_petit')
    cmd_vel_out_arg = DeclareLaunchArgument(
        'cmd_vel_out',
        default_value='/diff_drive_controller/cmd_vel_raw')
    # default_value=[LaunchConfiguration('robot_namespace'), '/diff_drive_controller/cmd_vel_raw'])

    container_name_arg = DeclareLaunchArgument(
        'container_name', default_value='teleop_container',
        description='the name of container that nodes will load in if use composition')

    return LaunchDescription([
        use_sim_time_arg,
        robot_namespace_arg,
        cmd_vel_out_arg,
        container_name_arg,
    ])
