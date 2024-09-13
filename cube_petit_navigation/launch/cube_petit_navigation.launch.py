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
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LoadComposableNodes
from launch_ros.actions import Node
from launch_ros.actions import SetParameter
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


def generate_launch_description() -> LaunchDescription:
    """Generate launch descriptions.

    Returns:
        Launch descriptions
    """
    pkg_share = pathlib.Path(FindPackageShare('cube_petit_navigation').find('cube_petit_navigation'))
    args = []
    args.append(DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'))
    args.append(DeclareLaunchArgument(
        'map',
        default_value=str(pathlib.Path.home() / 'map/gazebo_house/map.yaml'),
        description='Full path to map yaml file to load'))
    args.append(DeclareLaunchArgument(
        'keepout',
        default_value=LaunchConfiguration('map'),
        description='Full path to keepout yaml file to load'))
    args.append(DeclareLaunchArgument(
        'params_file',
        default_value=str(pkg_share / 'config/nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes'))

    args.append(DeclareLaunchArgument(
        'container_name', default_value='nav2_container',
        description='the name of container that nodes will load in if use composition'))

    nav2_bringup = pathlib.Path(FindPackageShare('nav2_bringup').find('nav2_bringup'))

    lifecycle_nodes = ['controller_server',
                       'smoother_server',
                       'planner_server',
                       'behavior_server',
                       'bt_navigator',
                       'waypoint_follower',
                       'keepout_mask_server',
                       'costmap_filter_info_server']
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]
    autostart = LaunchConfiguration('autostart', default='true')
    params_file = LaunchConfiguration('params_file')
    param_substitutions = {
        'autostart': autostart}

    configured_params = RewrittenYaml(
        source_file=params_file,
        param_rewrites=param_substitutions,
        convert_types=True)
    nav2 = GroupAction(
        actions=[
            SetParameter('use_sim_time', LaunchConfiguration('use_sim_time')),
            Node(
                name=LaunchConfiguration('container_name'),
                package='rclcpp_components',
                executable='component_container_isolated',
                parameters=[configured_params, {'autostart': autostart}],
                remappings=remappings,
                output='screen'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(nav2_bringup / 'launch/localization_launch.py')),
                launch_arguments={'map': LaunchConfiguration('map'),
                                  'params_file': params_file,
                                  'use_sim_time': LaunchConfiguration('use_sim_time'),
                                  'use_composition': 'True',
                                  'autostart': autostart,
                                  'container_name': LaunchConfiguration('container_name')}.items()),
            LoadComposableNodes(
                target_container=LaunchConfiguration('container_name'),
                composable_node_descriptions=[
                    ComposableNode(
                        package='nav2_controller',
                        plugin='nav2_controller::ControllerServer',
                        name='controller_server',
                        parameters=[configured_params],
                        remappings=remappings + [('cmd_vel', 'nav_vel')]),
                    ComposableNode(
                        package='nav2_smoother',
                        plugin='nav2_smoother::SmootherServer',
                        name='smoother_server',
                        parameters=[configured_params],
                        remappings=remappings),
                    ComposableNode(
                        package='nav2_planner',
                        plugin='nav2_planner::PlannerServer',
                        name='planner_server',
                        parameters=[configured_params],
                        remappings=remappings),
                    ComposableNode(
                        package='nav2_behaviors',
                        plugin='behavior_server::BehaviorServer',
                        name='behavior_server',
                        parameters=[configured_params],
                        remappings=remappings + [('cmd_vel', 'nav_vel')]),
                    ComposableNode(
                        package='nav2_bt_navigator',
                        plugin='nav2_bt_navigator::BtNavigator',
                        name='bt_navigator',
                        parameters=[configured_params],
                        remappings=remappings),
                    ComposableNode(
                        package='nav2_waypoint_follower',
                        plugin='nav2_waypoint_follower::WaypointFollower',
                        name='waypoint_follower',
                        parameters=[configured_params],
                        remappings=remappings),
                    ComposableNode(
                        package='nav2_lifecycle_manager',
                        plugin='nav2_lifecycle_manager::LifecycleManager',
                        name='lifecycle_manager_navigation',
                        parameters=[{'autostart': autostart,
                                     'node_names': lifecycle_nodes}]),
                ],
            ),
            Node(package='nav2_map_server',
                 executable='map_server',
                 name='keepout_mask_server',
                 output='screen',
                 emulate_tty=True,
                 parameters=[configured_params, {'yaml_filename': LaunchConfiguration('keepout')}]),
            Node(package='nav2_map_server',
                 executable='costmap_filter_info_server',
                 name='costmap_filter_info_server',
                 output='screen',
                 emulate_tty=True,
                 parameters=[configured_params])
        ]
    )

    return LaunchDescription(args + [
        nav2,
    ])
