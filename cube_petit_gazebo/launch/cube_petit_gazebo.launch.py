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
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import Node
from launch_ros.actions import PushROSNamespace
from launch_ros.actions import SetParameter
from launch_ros.actions import SetParametersFromFile
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


def select_world(context: LaunchContext, *args, **kwargs) -> list:
    sample_world = LaunchConfiguration('sample_world').perform(context)
    pkg_path = pathlib.Path(FindPackageShare('cube_petit_gazebo').find('cube_petit_gazebo'))

    if sample_world.lower() == 'true':
        world_path = pkg_path / 'worlds' / 'rooms.sdf'
    else:
        world_path = 'empty.sdf'

    gz_server = IncludeLaunchDescription([FindPackageShare('ros_gz_sim'), '/launch/gz_sim.launch.py'],
                                         launch_arguments={
                                             'gz_args': [str(world_path) + ' -v4 -s -r'],
                                             'on_exit_shutdown': 'true'
                                         }.items())
    return [gz_server]


def generate_launch_description() -> LaunchDescription:
    """Generate launch descriptions.

    Returns:
        Launch descriptions
    """
    print('gazebo generate launch dessciprtion')
    pkg_path = pathlib.Path(FindPackageShare('cube_petit_gazebo').find('cube_petit_gazebo'))
    share_path = pkg_path.parent
    gazebo_model_path = SetEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH',
                                               value=[
                                                   EnvironmentVariable('GZ_SIM_RESOURCE_PATH', default_value=''),
                                                   f':{pkg_path}',
                                                   f':{share_path}',
                                                   f':{pkg_path}/models',
                                               ])
    args = []
    args.append(DeclareLaunchArgument('robot', default_value='cube_petit_v3'))
    args.append(DeclareLaunchArgument('minimum', default_value='false'))

    args.append(DeclareLaunchArgument('robot_init_x', default_value='0.5'))
    args.append(DeclareLaunchArgument('robot_init_y', default_value='-2.0'))
    args.append(DeclareLaunchArgument('robot_init_yaw', default_value='0.0'))

    args.append(DeclareLaunchArgument('sample_world', default_value='false'))

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
    # [TODO] add microphone
    args.append(DeclareLaunchArgument('camera_manager', default_value='camera_container'))
    args.append(
        DeclareLaunchArgument('camera_config',
                              default_value=PathJoinSubstitution(
                                  [FindPackageShare('cube_petit_bringup'), 'config', 'realsense.yaml'])))
    print(LaunchConfiguration('camera_config'))
    camera_color_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'camera_optical_link', 'camera_color_optical_link'])

    camera_depth_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'camera_optical_link', 'camera_depth_optical_link'])

    realsense_group = GroupAction(actions=[
        PushROSNamespace('camera'),
        SetParametersFromFile(filename=LaunchConfiguration('camera_config')),
        ComposableNodeContainer(
            name=LaunchConfiguration('camera_manager'),
            namespace='',
            package='rclcpp_components',
            executable='component_container_mt',
            parameters=[{
                'use_sim_time': True
            }],
            composable_node_descriptions=[
                ComposableNode(
                    package='image_proc',
                    plugin='image_proc::RectifyNode',
                    name='high_resolution_rgb_rectify',
                    remappings=[
                        ('image', 'rgb_hi_res/image_raw'),
                        ('camera_info', 'rgb_hi_res/camera_info'),
                        ('image_rect', 'rgb_hi_res/image_rect'),
                    ],
                ),
                ComposableNode(
                    package='image_proc',
                    plugin='image_proc::ResizeNode',
                    name='rgb_resize',
                    remappings=[
                        ('image/image_raw', 'rgb_hi_res/image_raw'),
                        ('resize/image_raw', 'rgb/image_raw'),
                    ],
                ),
                ComposableNode(
                    package='image_proc',
                    plugin='image_proc::RectifyNode',
                    name='rgb_rectify',
                    remappings=[
                        ('image', 'rgb/image_raw'),
                        ('camera_info', 'rgb/camera_info'),
                        ('image_rect', 'rgb/image_rect'),
                    ],
                ),
                ComposableNode(
                    package='image_proc',
                    plugin='image_proc::RectifyNode',
                    name='depth_rectify',
                    remappings=[
                        ('image', 'depth/image_raw'),
                        ('camera_info', 'depth/camera_info'),
                        ('image_rect', 'depth/image_rect'),
                    ],
                ),
                ComposableNode(
                    package='depth_image_proc',
                    plugin='depth_image_proc::RegisterNode',
                    name='register',
                    remappings=[
                        ('image', 'depth/image_rect'),
                        ('camera_info', 'depth/camera_info'),
                        ('camera_info_rgb', 'rgb/camera_info'),
                        ('image_rect', 'depth_registered/image_rect'),
                    ],
                ),
                ComposableNode(
                    package='depth_image_proc',
                    plugin='depth_image_proc::PointCloudXyzrgbNode',
                    name='points_xyzrgb',
                    remappings=[
                        ('image_rect', 'depth_registered/image_rect'),
                        ('camera_info', 'depth/camera_info'),
                        ('rgb/image_rect_color', 'rgb/image_rect'),
                        ('points', 'depth_registered/points'),
                    ],
                ),
                ComposableNode(package='topic_tools',
                               plugin='topic_tools::ThrottleNode',
                               name='points_throttle',
                               parameters=[{
                                   'input_topic': 'depth_registered/points',
                                   'output_topic': 'depth_registered/points_throttled',
                                   'lazy': True,
                                   'throttle_type': 'messages',
                                   'msgs_per_sec': 5.0,
                               }]),
                ComposableNode(
                    package='pcl_ros',
                    plugin='pcl_ros::VoxelGrid',
                    name='voxel_grid',
                    remappings=[
                        ('input', 'depth_registered/points_throttled'),
                        ('output', 'depth_registered/points_voxel_filtered'),
                    ],
                ),
                ComposableNode(
                    package='pcl_ros',
                    plugin='pcl_ros::StatisticalOutlierRemoval',
                    name='statistical_outlier_removal',
                    remappings=[
                        ('input', 'depth_registered/points_voxel_filtered'),
                        ('output', 'depth_registered/cost_points'),
                    ],
                ),
                ComposableNode(
                    package='pcl_ros',
                    plugin='pcl_ros::PassThrough',
                    name='pointcloud_transformer',
                    remappings=[
                        ('input', 'depth_registered/points'),
                        ('output', 'depth_registered/points_fixed_frame'),
                    ],
                    parameters=[{
                        'filter_field_name': 'z',
                        'filter_limit_min': -0.10,
                        'filter_limit_max': 2.50,
                        'filter_limit_negative': False,
                        'input_frame': 'map',
                        'output_frame': 'map',
                    }],
                ),
            ],
        )
    ])
    return LaunchDescription([
        gazebo_model_path,
    ] + args + [
        SetParameter(name='use_sim_time', value=True),
        OpaqueFunction(function=select_world),
        camera_color_tf,
        camera_depth_tf,
        OpaqueFunction(function=launch_setup),
        gz_client,
        teleop,
        realsense_group,
        # text_to_speech,
        # hotword,
    ])
