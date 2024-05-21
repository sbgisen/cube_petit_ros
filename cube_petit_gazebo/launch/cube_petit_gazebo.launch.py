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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.actions import SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_context import LaunchContext
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetParameter
from launch_ros.substitutions import FindPackageShare


def launch_setup(context: LaunchContext, *args, **kwargs) -> list:
    print("gazebo launch launch setup")

    pkg_path = pathlib.Path(FindPackageShare('cube_petit_gazebo').find('cube_petit_gazebo'))
    return [IncludeLaunchDescription([str(pkg_path / 'launch/include'), '/',
                                      LaunchConfiguration('robot').perform(context), '_gazebo.launch.py'],
                                     launch_arguments={'minimum': LaunchConfiguration('minimum'),
                                                       'x': LaunchConfiguration('robot_init_x'),
                                                       'y': LaunchConfiguration('robot_init_y'),
                                                       'z': '0.0',
                                                       'roll': '0.0',
                                                       'pitch': '0.0',
                                                       'yaw': LaunchConfiguration('robot_init_yaw')}.items())]


def generate_launch_description() -> LaunchDescription:
    """Generate launch descriptions.

    Returns:
        Launch descriptions
    """
    print("gazebo generate launch dessciprtion")
    pkg_path = pathlib.Path(FindPackageShare('cube_petit_gazebo').find('cube_petit_gazebo'))
    gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=[EnvironmentVariable('GAZEBO_MODEL_PATH', default_value=''), f':{pkg_path}/models'])

    pkg_gazebo_ros = pathlib.Path(FindPackageShare('gazebo_ros').find('gazebo_ros'))

    args = []
    args.append(DeclareLaunchArgument(
        'robot',
        default_value='cube_petit_v3'))
    args.append(DeclareLaunchArgument(
        'debug',
        default_value='false'))
    args.append(DeclareLaunchArgument(
        'verbose',
        default_value='false'))
    args.append(DeclareLaunchArgument(
        'gui',
        default_value='true'))
    args.append(DeclareLaunchArgument(
        'paused',
        default_value='false'))
    args.append(DeclareLaunchArgument(
        'world',
        default_value='sample'))
    args.append(DeclareLaunchArgument(
        'minimum',
        default_value='false'))
    args.append(DeclareLaunchArgument(
        'robot_init_x',
        default_value='-2.5',
        description='X position of the robot in the Gazebo world'))
    args.append(DeclareLaunchArgument(
        'robot_init_y',
        default_value='0.0',
        description='Y position of the robot in the Gazebo world'))
    args.append(DeclareLaunchArgument(
        'robot_init_yaw',
        default_value='0.0',
        description='Yaw of the robot in the Gazebo world'))
    args.append(DeclareLaunchArgument(
        'station_x',
        default_value='-2.393',
        description='X position of the robot in the Gazebo world'))
    args.append(DeclareLaunchArgument(
        'station_y',
        default_value='8.494',
        description='Y position of the robot in the Gazebo world'))
    args.append(DeclareLaunchArgument(
        'station_yaw',
        default_value='-1.5708',
        description='Yaw of the robot in the Gazebo world'))

    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(pkg_gazebo_ros / 'launch/gzserver.launch.py')
        ),
        launch_arguments={'world': [str(pkg_path / "worlds"), '/', LaunchConfiguration('world'), '.world'],
                          'gui': LaunchConfiguration('gui'),
                          'debug': LaunchConfiguration('debug'),
                          'verbose': LaunchConfiguration('verbose'),
                          'paused': LaunchConfiguration('paused'),
                          'use_sim_time': 'true'}.items()
    )

    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(pkg_gazebo_ros / 'launch/gzclient.launch.py')
        )
    )

    return LaunchDescription(args + [
        SetParameter(name='use_sim_time', value=True),
        gazebo_model_path,
        gzserver,
        gzclient,
        OpaqueFunction(function=launch_setup),
    ])
