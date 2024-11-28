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

import os
import pathlib

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
from launch_ros.actions import SetParameter
from launch_ros.substitutions import FindPackageShare
import xacro
import yaml


def generate_launch_description() -> LaunchDescription:
    """Generate launch descriptions.

    Returns:
        Launch descriptions
    """
    args = []
    args.append(DeclareLaunchArgument('robot', default_value='cube_petit'))
    args.append(DeclareLaunchArgument('minimum', default_value='false'))
    args.append(
        DeclareLaunchArgument('x', default_value='0.0', description='X position of the robot in the Gazebo world'))
    args.append(
        DeclareLaunchArgument('y', default_value='0.0', description='Y position of the robot in the Gazebo world'))
    args.append(
        DeclareLaunchArgument('z', default_value='0.0', description='Z position of the robot in the Gazebo world'))
    args.append(DeclareLaunchArgument('roll', default_value='0.0',
                                      description='Roll of the robot in the Gazebo world'))
    args.append(
        DeclareLaunchArgument('pitch', default_value='0.0', description='Pitch of the robot in the Gazebo world'))
    args.append(DeclareLaunchArgument('yaw', default_value='0.0', description='Yaw of the robot in the Gazebo world'))

    pkg_path = pathlib.Path(FindPackageShare('cube_petit_gazebo').find('cube_petit_gazebo'))

    description_pkg = FindPackageShare('cube_petit_description').find('cube_petit_description')
    print(description_pkg)
    xacro_file = pathlib.Path(description_pkg) / 'xacro/cube_petit_gazebo.xacro'

    doc = xacro.process_file(xacro_file, mappings={'use_sim': 'true'})
    robot_description = doc.toprettyxml(indent='  ')
    print(robot_description)
    config_path = os.path.join(pkg_path, 'config', 'gz_bridge.yaml')
    with open(config_path, 'r') as f:
        gz_bridge_params = yaml.safe_load(f)
        with open('/tmp/gz_bridge.yaml', 'w') as f2:
            f2.write(yaml.safe_dump(gz_bridge_params))
    spawn_entity = Node(package='ros_gz_sim',
                        executable='create',
                        arguments=[
                            '-name',
                            LaunchConfiguration('robot'),
                            '-string',
                            robot_description,
                            '-x',
                            '0.0',
                            '-y',
                            '0.0',
                            '-Y',
                            '0.0',
                        ])
    param_bridge = Node(package='ros_gz_bridge',
                        executable='parameter_bridge',
                        arguments=[
                            '--ros-args',
                            '-p',
                            'config_file:=/tmp/gz_bridge.yaml',
                        ])
    robot_state_publisher = Node(package='robot_state_publisher',
                                 executable='robot_state_publisher',
                                 output='both',
                                 parameters=[{
                                     'robot_description': robot_description
                                 }])

    hardware_pkg = pathlib.Path(FindPackageShare('cube_petit_gazebo').find('cube_petit_gazebo'))
    controllers = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        [str(hardware_pkg / 'launch/include/base_control_gazebo.launch.py')]),
                                           launch_arguments={
                                               'robot': LaunchConfiguration('robot'),
                                               'x': LaunchConfiguration('x'),
                                               'y': LaunchConfiguration('y'),
                                               'z': LaunchConfiguration('z'),
                                               'roll': LaunchConfiguration('roll'),
                                               'pitch': LaunchConfiguration('pitch'),
                                               'yaw': LaunchConfiguration('yaw'),
                                               'minimum': LaunchConfiguration('minimum')
                                           }.items())

    load_joint_state_controller = Node(package='controller_manager',
                                       executable='spawner',
                                       output='both',
                                       arguments=['-c', '/controller_manager', 'joint_state_broadcaster'])

    load_diff_drive_controller = Node(package='controller_manager',
                                      executable='spawner',
                                      output='both',
                                      arguments=['-c', '/controller_manager', 'diff_drive_controller'])
    image_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=[
            ['/camera/rgb/image_raw'],
            ['/camera/depth/image_raw'],
        ],
    )

    return LaunchDescription(args + [
        SetParameter(name='use_sim_time', value=True),
        robot_state_publisher,
        RegisterEventHandler(
            event_handler=OnProcessExit(target_action=robot_state_publisher, on_exit=[EmitEvent(event=Shutdown())])),
        spawn_entity,
        load_joint_state_controller,
        load_diff_drive_controller,
        controllers,
        param_bridge,
        image_bridge,
    ])
