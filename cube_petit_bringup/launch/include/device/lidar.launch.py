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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

# def launch_setup(context: LaunchContext, *args, **kwargs) -> list:
#     bringup_pkg = pathlib.Path(FindPackageShare('cube_petit_bringup').find('cube_petit_bringup'))
#     laser_filter = bringup_pkg / 'config/sensors' / LaunchConfiguration('yaml_file').perform(context)
#     topic = LaunchConfiguration('scan_topic').perform(context)
#     with open(laser_filter) as f:
#         laser_filter_params = yaml.safe_load(f)
#     return [Node(package='laser_filters', executable='scan_to_scan_filter_chain',
#                  name=['lh_laser_node_', LaunchConfiguration('name'), '_filter'],
#                  remappings=[('scan', f'{topic}_raw'), ('scan_filtered', topic)],
#                  parameters=[laser_filter_params])]


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

    ldlidar_node = Node(
        package='ldlidar_stl_ros2',
        executable='ldlidar_stl_ros2_node',
        name='LD06',
        output='screen',
        parameters=[
            {'product_name': 'LDLiDAR_LD06'},
            {'topic_name': 'scan'},
            {'frame_id': 'base_laser'},
            {'port_name': '/dev/ttyLD06-19'},
            {'port_baudrate': 230400},
            {'laser_scan_dir': True},
            {'enable_angle_crop_func': False},
            # {'angle_crop_min': 135.0},
            # {'angle_crop_max': 225.0}
        ]
    )

    # base_link to base_laser tf node
    base_link_to_laser_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_base_laser_ld19',
        arguments=['0', '0', '0.18', '0', '0', '0', 'base_link', 'ld19_link']
    )

    return LaunchDescription(args + [
        # laser,
        # OpaqueFunction(function=launch_setup),
        ldlidar_node,
        base_link_to_laser_tf_node,
    ])
