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
from distutils.util import strtobool

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.actions import TimerAction
from launch.launch_context import LaunchContext
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context: LaunchContext, *args, **kwargs) -> list:
    bringup_pkg = pathlib.Path(FindPackageShare('cube_petit_bringup').find('cube_petit_bringup'))
    launch_robot_path = PathJoinSubstitution([str(bringup_pkg), 'launch/include', 'cube_petit_v3'])

    teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([str(bringup_pkg),
                                       '/launch/include/',
                                       LaunchConfiguration('robot'),
                                       '/velocity_commands.launch.py']),
        launch_arguments={'robot_namespace': LaunchConfiguration('robot_namespace')}.items())

    if strtobool(LaunchConfiguration('gazebo').perform(context)):
        return [TimerAction(period=5.0, actions=[teleop])]

    return [GroupAction(actions=[IncludeLaunchDescription(
                        PythonLaunchDescriptionSource([launch_robot_path, '/sensors.launch.py']),
                        launch_arguments={
                            'robot_namespace': LaunchConfiguration('robot_namespace')}.items())])]


def generate_launch_description() -> LaunchDescription:
    """Generate launch descriptions.

    Returns:
        Launch descriptions
    """
    args = []
    args.append(DeclareLaunchArgument(
        'robot',
        default_value='cube_petit_v3'))
    args.append(DeclareLaunchArgument(
        'robot_namespace',
        default_value=LaunchConfiguration('robot')))
    args.append(DeclareLaunchArgument(
        'disable_ros_controller',
        description='Disable basic ros controller to use customize ros controller.',
        default_value='false'))
    description_pkg = FindPackageShare('cube_description').find('cube_description')
    args.append(DeclareLaunchArgument(
        'hardware_config',
        default_value=str(pathlib.Path(description_pkg) / 'xacro/cuboid_robot.xacro')))
    bringup_pkg = pathlib.Path(FindPackageShare('cube_bringup').find('cube_bringup'))
    args.append(DeclareLaunchArgument(
        'ekf_config',
        default_value=str(bringup_pkg / 'config/ekf.yaml')))
    args.append(DeclareLaunchArgument(
        'gazebo',
        default_value='false'))

    # TODO: speech
    expression_pkg = pathlib.Path(FindPackageShare('cube_expression').find('cube_expression'))
    face = IncludeLaunchDescription(PythonLaunchDescriptionSource(str(expression_pkg
                                                                      / 'launch/cube_expression.launch.py')),
                                    launch_arguments={'gazebo': LaunchConfiguration('gazebo')}.items())

    hardware_pkg = pathlib.Path(FindPackageShare('cube_petit_hardware_interface').find('cube_petit_hardware_interface'))
    motor_bringup = IncludeLaunchDescription(PythonLaunchDescriptionSource(str(hardware_pkg
                                                                      / 'launch/cube_petit_control.launch.py')),
                                    launch_arguments={}.items())

    robot_localization = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_odom',
        output='screen',
        parameters=[LaunchConfiguration('ekf_config')],
        remappings=[('odometry/filtered', 'odom')])

    return LaunchDescription(args + [
        motor_bringup,
        robot_localization,
        OpaqueFunction(function=launch_setup)
    ])
