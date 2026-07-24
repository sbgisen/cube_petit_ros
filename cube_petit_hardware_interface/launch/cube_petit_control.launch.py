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

from distutils.util import strtobool
import os
import pathlib

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.launch_context import LaunchContext
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
import xacro


def launch_setup(context: LaunchContext, *args, **kwargs) -> list:
    description_pkg = FindPackageShare('cube_petit_description').find('cube_petit_description')
    xacro_file = os.path.join(description_pkg, 'xacro', 'cube_petit.xacro')
    use_sim = 'false'  # or 'false', depending on your use case
    doc = xacro.process_file(xacro_file, mappings={'use_sim': use_sim})
    robot_description = {'robot_description': doc.toprettyxml(indent='  ')}

    controllers_yaml = os.path.join(
        get_package_share_directory('cube_petit_hardware_interface'),
        'config',
        'cube_petit_hw_interface.yaml',
    )

    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, controllers_yaml],
        output='both',
    )

    if strtobool(LaunchConfiguration('disable_ros_controller').perform(context)):
        return [control_node]

    controllers = GroupAction(actions=[
        Node(package='controller_manager',
             executable='spawner',
             output='both',
             arguments=[
                 'joint_state_broadcaster',
                 '--controller-manager',
                 'controller_manager',
             ]),
        Node(package='controller_manager',
             executable='spawner',
             output='both',
             arguments=[
                 'diff_drive_controller',
                 '--controller-manager',
                 'controller_manager',
             ]),
    ])

    return [control_node, controllers]


def generate_launch_description() -> LaunchDescription:
    """Generate launch descriptions.

    Returns:
        Launch descriptions
    """
    args = []
    args.append(DeclareLaunchArgument('robot', default_value='cube_petit'))
    args.append(DeclareLaunchArgument('robot_namespace', default_value=LaunchConfiguration('robot')))
    args.append(
        DeclareLaunchArgument('disable_ros_controller',
                              description='Disable basic ros controller to use customize ros controller.',
                              default_value='false'))

    description_pkg = FindPackageShare('cube_petit_description').find('cube_petit_description')
    args.append(
        DeclareLaunchArgument('hardware_config',
                              default_value=str(pathlib.Path(description_pkg) / 'xacro/cube_petit.xacro')))

    socketcan_bridge_pkg = pathlib.Path(FindPackageShare('ros2_socketcan').find('ros2_socketcan'))
    socketcan_bridge = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(str(socketcan_bridge_pkg / 'launch/socket_can_bridge.launch.xml')),
        # ttyCANable
        launch_arguments={
            'interface': 'can0',
            'receiver_interval_sec': '0.1',
            'sender_timeout_sec': '0.01',
            'enable_can_fd': 'false',
            'from_can_bus_topic': 'from_can_bus',
            'to_can_bus_topic': 'to_can_bus'
        }.items())

    description_pkg = FindPackageShare('cube_petit_description').find('cube_petit_description')
    args.append(
        DeclareLaunchArgument('hardware_config',
                              default_value=str(pathlib.Path(description_pkg) / 'xacro/cube_petit.xacro')))

    return LaunchDescription(args + [
        OpaqueFunction(function=launch_setup),
        socketcan_bridge,
        # hardware_interface,
    ])
