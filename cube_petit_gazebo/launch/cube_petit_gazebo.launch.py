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
from launch.launch_context import LaunchContext
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import PushROSNamespace
from launch_ros.actions import SetParameter
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def launch_setup(context: LaunchContext, *args, **kwargs) -> list:
    print('gazebo launch launch setup')

    pkg_path = pathlib.Path(FindPackageShare('cube_petit_gazebo').find('cube_petit_gazebo'))
    return [
        IncludeLaunchDescription(
            [
                str(pkg_path / 'launch/include'), '/',
                LaunchConfiguration('robot').perform(context), '_gazebo.launch.py'
            ],
            launch_arguments={
                'minimum': LaunchConfiguration('minimum'),
                'x': LaunchConfiguration('robot_init_x'),
                'y': LaunchConfiguration('robot_init_y'),
                'z': '0.0',
                'roll': '0.0',
                'pitch': '0.0',
                'yaw': LaunchConfiguration('robot_init_yaw')
            }.items())
    ]


def generate_launch_description() -> LaunchDescription:
    """Generate launch descriptions.

    Returns:
        Launch descriptions
    """
    print('gazebo generate launch dessciprtion')
    pkg_path = pathlib.Path(FindPackageShare('cube_petit_gazebo').find('cube_petit_gazebo'))
    gazebo_model_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[EnvironmentVariable('GZ_SIM_RESOURCE_PATH', default_value=''), f':{pkg_path}/models'])

    args = []
    args.append(DeclareLaunchArgument('robot', default_value='cube_petit_v3'))
    args.append(DeclareLaunchArgument('minimum', default_value='false'))

    args.append(DeclareLaunchArgument('robot_init_x', default_value='0.5'))
    args.append(DeclareLaunchArgument('robot_init_y', default_value='-2.0'))
    args.append(DeclareLaunchArgument('robot_init_yaw', default_value='0.0'))

    gz_server = IncludeLaunchDescription(
        [FindPackageShare('ros_gz_sim'), '/launch/gz_sim.launch.py'],
        launch_arguments={
            'gz_args': ['world /home/gisen/ros/src/cube_petit_ros/cube_petit_gazebo/worlds/sample.sdf -r -s -v4'],
            'on_exit_shutdown': 'true'
        }.items())

    gz_client = IncludeLaunchDescription([FindPackageShare('ros_gz_sim'), '/launch/gz_sim.launch.py'],
                                         launch_arguments={'gz_args': '-g -v4 '}.items())

    # [TODO]
    bringup_pkg = pathlib.Path(FindPackageShare('cube_petit_bringup').find('cube_petit_bringup'))
    teleop = IncludeLaunchDescription(PythonLaunchDescriptionSource(str(bringup_pkg / 'launch/teleop.launch.py')))

    # text_to_speech_pkg = pathlib.Path(FindPackageShare('cube_petit_text_to_speech').find('cube_petit_text_to_speech')
    # text_to_speech = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(str(text_to_speech_pkg / 'launch/cube_petit_text_to_jtalk.launch.py')))

    # speech_to_text_pkg = pathlib.Path(FindPackageShare('cube_petit_speech_to_text').find('cube_petit_speech_to_text')
    # hotword = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(str(speech_to_text_pkg / 'launch/cube_petit_hotword.launch.py')))

    depth_camera = GroupAction(actions=[
        ComposableNodeContainer(
            name='camera_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='image_proc',
                    plugin='image_proc::RectifyNode',
                    name='rgb_rectify',
                    remappings=[
                        ('image', '/camera/rgb/image_raw'),
                        ('camera_info', '/camera/rgb/camera_info'),
                        ('image_rect', '/camera/rgb/image_rect'),
                    ],
                ),
                ComposableNode(
                    package='image_proc',
                    plugin='image_proc::RectifyNode',
                    name='depth_rectify',
                    remappings=[
                        ('image', '/camera/depth/image_raw'),
                        ('camera_info', '/camera/depth/camera_info'),
                        ('image_rect', '/camera/depth/image_rect'),
                    ],
                ),
                ComposableNode(
                    package='depth_image_proc',
                    plugin='depth_image_proc::RegisterNode',
                    name='register',
                    remappings=[
                        ('depth/image_rect', '/camera/depth/image_rect'),
                        ('rgb/camera_info', '/camera/rgb/camera_info'),
                        ('depth/camera_info', '/camera/depth/camera_info'),
                        # ('depth_registered/image_rect',
                        #  '/camera/depth_registered/image_rect'),
                        # ('depth_registered/camera_info',
                        #  '/camera/depth_registered/camera_info'),
                    ],
                ),
                ComposableNode(
                    package='depth_image_proc',
                    plugin='depth_image_proc::PointCloudXyzrgbNode',
                    name='points_xyzrgb',
                    remappings=[
                        ('depth_registered/image_rect', '/camera/depth_registered/image_rect'),
                        ('rgb/image_rect_color', '/camera/rgb/image_rect'),
                        ('rgb/camera_info', '/camera/rgb/camera_info'),
                        ('points', '/camera/depth/points'),
                    ],
                ),
            ],
        )
    ])

    return LaunchDescription(args + [
        SetParameter(name='use_sim_time', value=True),
        gazebo_model_path,
        gz_server,
        gz_client,
        OpaqueFunction(function=launch_setup),
        teleop,
        depth_camera,
        # text_to_speech,
        # hotword,
    ])
