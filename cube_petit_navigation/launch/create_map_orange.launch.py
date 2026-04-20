#!/usr/bin/env python

# Copyright (c) 2025 SoftBank Corp.
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
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import PushRosNamespace
from launch_ros.actions import SetParameter
from launch_ros.actions import SetParametersFromFile
from launch_ros.actions import SetRemap
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
    args.append(
        DeclareLaunchArgument('use_sim_time',
                              default_value='false',
                              description='Use simulation (Gazebo) clock if true'))
    args.append(
        DeclareLaunchArgument('params_file',
                              default_value=str(pkg_share / 'config/slam_orange.yaml'),
                              description='Full path to the ROS2 parameters file to use for all launched nodes'))

    args.append(
        DeclareLaunchArgument('container_name',
                              default_value='nav2_container',
                              description='the name of container that nodes will load in if use composition'))
    args.append(DeclareLaunchArgument('robot', default_value='cube_petit_orange'))

    lifecycle_nodes = [
        'controller_server', 'smoother_server', 'planner_server', 'behavior_server', 'bt_navigator',
        'waypoint_follower'
    ]

    autostart = LaunchConfiguration('autostart', default='true')
    params_file = LaunchConfiguration('params_file')
    param_substitutions = {
        'autostart': autostart,
    }
    configured_params = RewrittenYaml(source_file=params_file,
                                      root_key=[LaunchConfiguration('robot'), '/navigation'],
                                      param_rewrites=param_substitutions,
                                      convert_types=False)

    slam_launch_file = pathlib.Path(
        FindPackageShare('slam_toolbox').find('slam_toolbox')) / 'launch' / 'online_async_launch.py'

    nav2 = GroupAction(actions=[
        PushRosNamespace([LaunchConfiguration('robot'), '/navigation']),
        SetParameter('use_sim_time', LaunchConfiguration('use_sim_time')),
        SetParametersFromFile(configured_params),
        SetRemap('/map', 'map'),
        SetRemap('/map_metadata', 'map_metadata'),
        GroupAction(actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(slam_launch_file)),
                launch_arguments={
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'slam_params_file': configured_params
                }.items(),
            )
        ]),
        ComposableNodeContainer(
            name=LaunchConfiguration('container_name'),
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(package='nav2_controller',
                               plugin='nav2_controller::ControllerServer',
                               name='controller_server',
                               remappings=[
                                   ('odom', ['/', LaunchConfiguration('robot'), '/odom']),
                                   ('scan', ['/', LaunchConfiguration('robot'), '/laser/scan']),
                               ]),
                ComposableNode(package='nav2_smoother', plugin='nav2_smoother::SmootherServer',
                               name='smoother_server'),
                ComposableNode(package='nav2_planner', plugin='nav2_planner::PlannerServer', name='planner_server'),
                ComposableNode(
                    package='nav2_behaviors', plugin='behavior_server::BehaviorServer', name='behavior_server'),
                ComposableNode(package='nav2_bt_navigator',
                               plugin='nav2_bt_navigator::BtNavigator',
                               name='bt_navigator',
                               remappings=[('odom', ['/', LaunchConfiguration('robot'), '/odom'])]),
                ComposableNode(package='nav2_waypoint_follower',
                               plugin='nav2_waypoint_follower::WaypointFollower',
                               name='waypoint_follower'),
                ComposableNode(package='nav2_lifecycle_manager',
                               plugin='nav2_lifecycle_manager::LifecycleManager',
                               name='lifecycle_manager_navigation',
                               parameters=[{
                                   'autostart': autostart,
                                   'node_names': lifecycle_nodes
                               }]),
            ]),
    ])
    
    bridge_base_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_bridge',
        arguments=[
            '0', '0', '0',
            '0', '0', '0',
            'cube_petit_orange/base_link',
            'base_link',
        ],
        output='screen',
    )

    return LaunchDescription(args + [
        nav2,
        bridge_base_link,
    ])
