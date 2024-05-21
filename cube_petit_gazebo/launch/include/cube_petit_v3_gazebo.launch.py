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
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import EmitEvent
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.actions import SetParameter
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
    args.append(DeclareLaunchArgument(
        'minimum',
        default_value='false'))
    args.append(DeclareLaunchArgument(
        'x',
        default_value='0.0',
        description='X position of the robot in the Gazebo world'))
    args.append(DeclareLaunchArgument(
        'y',
        default_value='0.0',
        description='Y position of the robot in the Gazebo world'))
    args.append(DeclareLaunchArgument(
        'z',
        default_value='0.0',
        description='Z position of the robot in the Gazebo world'))
    args.append(DeclareLaunchArgument(
        'roll',
        default_value='0.0',
        description='Roll of the robot in the Gazebo world'))
    args.append(DeclareLaunchArgument(
        'pitch',
        default_value='0.0',
        description='Pitch of the robot in the Gazebo world'))
    args.append(DeclareLaunchArgument(
        'yaw',
        default_value='0.0',
        description='Yaw of the robot in the Gazebo world'))

    pkg_path = pathlib.Path(FindPackageShare('cube_petit_gazebo').find('cube_petit_gazebo'))

    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    roll = LaunchConfiguration('roll')
    pitch = LaunchConfiguration('pitch')
    yaw = LaunchConfiguration('yaw')
    spawn_entity = Node(package='gazebo_ros',
                        executable='spawn_entity.py',
                        arguments=['-entity', LaunchConfiguration('robot'),
                                   '-topic', 'robot_description',
                                   '-x', x,
                                   '-y', y,
                                   '-z', z,
                                   '-R', roll,
                                   '-P', pitch,
                                   '-Y', yaw],
                        output='screen')

    description_pkg = FindPackageShare('cube_petit_description').find('cube_petit_description')
    print(description_pkg)
    xacro_file = pathlib.Path(description_pkg) / 'xacro/cube_petit.xacro'

    doc = xacro.process_file(xacro_file, mappings={'use_sim': 'true'})
    robot_description = doc.toprettyxml(indent='  ')
    print(robot_description)

    robot_state_publisher = Node(package='robot_state_publisher',
                                 executable='robot_state_publisher',
                                 output='both',
                                 parameters=[{'robot_description': robot_description}])


    hardware_pkg = pathlib.Path(FindPackageShare('cube_petit_gazebo').find('cube_petit_gazebo'))
    controllers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([str(hardware_pkg / 'launch/include/base_control_gazebo.launch.py')]),
        launch_arguments={'robot': LaunchConfiguration('robot'),
                          'x': LaunchConfiguration('x'),
                          'y': LaunchConfiguration('y'),
                          'z': LaunchConfiguration('z'),
                          'roll': LaunchConfiguration('roll'),
                          'pitch': LaunchConfiguration('pitch'),
                          'yaw': LaunchConfiguration('yaw'),
                          'minimum': LaunchConfiguration('minimum')}.items())

    controller_config_path = pkg_path / 'config/base_control_gazebo.yaml'
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{'robot_description': robot_description}, controller_config_path],
        output='screen'
    )
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )
    diff_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller'],
        output='screen'
    )

    return LaunchDescription(args + [
        SetParameter(name='use_sim_time', value=True),
        robot_state_publisher,
        RegisterEventHandler(event_handler=OnProcessExit(
            target_action=robot_state_publisher,
            on_exit=[EmitEvent(event=Shutdown())]
        )),
        spawn_entity,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[controllers],
            )
        ),
        ros2_control_node,
        RegisterEventHandler(
            OnProcessExit(
                target_action=ros2_control_node,
                on_exit=[joint_state_broadcaster_spawner]
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[diff_drive_controller_spawner]
            )
        )


    ])
