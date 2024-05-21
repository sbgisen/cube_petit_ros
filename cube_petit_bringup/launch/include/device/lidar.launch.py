#!/usr/bin/env python
# -*- coding:utf-8 -*-

# Copyright (c) 2022 SoftBank Corp.
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

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.launch_context import LaunchContext
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context: LaunchContext, *args, **kwargs) -> list:
    bringup_pkg = pathlib.Path(FindPackageShare('cube_petit_bringup').find('cube_petit_bringup'))
    laser_filter = bringup_pkg / 'config' / LaunchConfiguration('yaml_file').perform(context)
    topic = LaunchConfiguration('scan_topic').perform(context)
    with open(laser_filter) as f:
        laser_filter_params = yaml.safe_load(f)
    return [Node(package='laser_filters', executable='scan_to_scan_filter_chain',
                 name=['lh_laser_node_', LaunchConfiguration('name'), '_filter'],
                 remappings=[('scan', f'{topic}_raw'), ('scan_filtered', topic)],
                 parameters=[laser_filter_params])]


def generate_launch_description() -> LaunchDescription:
    """Generate launch descriptions.

    Returns:
        Launch descriptions
    """
    args = []
    args.append(DeclareLaunchArgument(
        'name',
        default_value='pacecat'))
    args.append(DeclareLaunchArgument(
        'port',
        default_value='/dev/ttyLDS50C'))
    args.append(DeclareLaunchArgument(
        'frame',
        default_value='pacecat_laser_link'))
    args.append(DeclareLaunchArgument(
        'scan_topic',
        default_value='pacecat_scan'))
    args.append(DeclareLaunchArgument(
        'yaml_file',
        default_value='lidar_scan_filter.yaml'))

    laser = Node(package='lh_laser_driver',
                 executable='lh_laser_publisher',
                 name=['lh_laser_node_', LaunchConfiguration('name')],
                 parameters=[{'port': LaunchConfiguration('port'),
                              'frame_id': LaunchConfiguration('frame')}],
                 remappings=[('scan', [LaunchConfiguration('scan_topic'), '_raw'])],
                 output='screen')

    return LaunchDescription(args + [
        laser,
        OpaqueFunction(function=launch_setup),
    ])
