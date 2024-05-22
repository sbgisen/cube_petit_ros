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
from distutils.util import strtobool

import xacro
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.actions import OpaqueFunction
from launch.launch_context import LaunchContext
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context: LaunchContext, *args, **kwargs) -> list:

    doc = xacro.process_file(LaunchConfiguration('hardware_config').perform(context), mappings={'use_sim': 'false'})
    robot_description = {"robot_description": doc.toprettyxml(indent='  ')}

    my_pkg = FindPackageShare('cube_petit_hardware_interface').find('cube_petit_hardware_interface')
    robot_controllers = [my_pkg, '/config/cube_petit_hw_interface.yaml']

    control_node = GroupAction(actions=[
        # PushRosNamespace(LaunchConfiguration('robot_namespace')),
        Node(package="controller_manager",
             executable="ros2_control_node",
             parameters=[robot_description, robot_controllers],
             output="both",
             )])

    if strtobool(LaunchConfiguration('disable_ros_controller').perform(context)):
        return [control_node]

    controllers = GroupAction(actions=[
        # PushRosNamespace(LaunchConfiguration('robot_namespace')),
        Node(package='controller_manager',
             executable='spawner',
             output='both',
             arguments=["--controller-manager", "controller_manager",
                        'joint_state_broadcaster']),
        Node(package='controller_manager',
             executable='spawner',
             output='both',
             arguments=["--controller-manager", "controller_manager",
                        'diff_drive_controller'])])
    return [control_node, controllers]


def generate_launch_description() -> LaunchDescription:
    """Generate launch descriptions.

    Returns:
        Launch descriptions
    """
    args = []
    args.append(DeclareLaunchArgument(
        'robot',
        default_value='cube_petit'))
    args.append(DeclareLaunchArgument(
        'robot_namespace',
        default_value=LaunchConfiguration('robot')))
    args.append(DeclareLaunchArgument(
        'disable_ros_controller',
        description='Disable basic ros controller to use customize ros controller.',
        default_value='false'))

    description_pkg = FindPackageShare('cube_petit_description').find('cube_petit_description')
    args.append(DeclareLaunchArgument(
        'hardware_config',
        default_value=str(pathlib.Path(description_pkg) / 'xacro/cube_petit.xacro')))

    return LaunchDescription(args + [
        OpaqueFunction(function=launch_setup)
    ])
