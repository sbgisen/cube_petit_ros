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

# !/usr/bin/env python

# Copyright (c) 2022 SoftBank Corp.
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
from launch.actions import OpaqueFunction
from launch.launch_context import LaunchContext
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace
from launch_ros.actions import SetParameter
from launch_ros.actions import SetParametersFromFile
from launch_ros.actions import SetRemap
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


def launch_setup(context: LaunchContext, *args, **kwargs) -> list:
    actions = []
    map_file = LaunchConfiguration('map').perform(context)
    keepout_file = LaunchConfiguration('keepout')

    lifecycle_nodes = [
        'controller_server',
        'smoother_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower',
        'keepout_mask_server',
        'costmap_filter_info_server',
    ]
    autostart = LaunchConfiguration('autostart', default='true')
    params_file = LaunchConfiguration('params_file')
    param_substitutions = {
        'autostart': autostart,
        'filter_info_topic': ['/', LaunchConfiguration('robot'), '/navigation/costmap_filter_info'],
    }

    configured_params = RewrittenYaml(source_file=params_file,
                                      root_key=[LaunchConfiguration('robot'), '/navigation'],
                                      param_rewrites=param_substitutions,
                                      convert_types=True)

    actions = [
        PushRosNamespace([LaunchConfiguration('robot'), '/navigation']),
        SetParameter('use_sim_time', LaunchConfiguration('use_sim_time')),
        SetParametersFromFile(configured_params),
        SetRemap('/laser/scan', ['/', LaunchConfiguration('robot'), '/laser/scan']),
        SetRemap('/camera/depth_registered/cost_points',
                 ['/', LaunchConfiguration('robot'), '/camera/depth_registered/cost_points']),
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
                               name='waypoint_follower',
                               remappings=[('/fromLL', 'fromLL')]),
                ComposableNode(package='nav2_lifecycle_manager',
                               plugin='nav2_lifecycle_manager::LifecycleManager',
                               name='lifecycle_manager_navigation',
                               parameters=[{
                                   'autostart': autostart,
                                   'node_names': lifecycle_nodes
                               }]),
                ComposableNode(package='nav2_map_server',
                               plugin='nav2_map_server::MapServer',
                               name='map_server',
                               parameters=[{
                                   'yaml_filename': map_file
                               }]),
                ComposableNode(package='emcl2',
                               plugin='emcl2::EMcl2Node',
                               name='emcl',
                               parameters=[params_file],
                               remappings=[('scan', ['/', LaunchConfiguration('robot'), '/laser/scan'])]),
                ComposableNode(package='nav2_lifecycle_manager',
                               plugin='nav2_lifecycle_manager::LifecycleManager',
                               name='lifecycle_manager_localization',
                               parameters=[{
                                   'autostart': autostart,
                                   'node_names': ['map_server', 'emcl']
                               }]),
            ]),
        Node(package='nav2_map_server',
             executable='map_server',
             name='keepout_mask_server',
             output='screen',
             emulate_tty=True,
             parameters=[{
                 'yaml_filename': keepout_file
             }]),
        Node(package='nav2_map_server',
             executable='costmap_filter_info_server',
             name='costmap_filter_info_server',
             output='screen',
             emulate_tty=True,
             parameters=[{
                 'mask_topic': ['/', LaunchConfiguration('robot'), '/navigation/keepout_mask']
             }]),
    ] + actions

    return [GroupAction(actions=actions)]


def generate_launch_description() -> LaunchDescription:
    """Generate launch descriptions.

    Returns:
        Launch descriptions
    """
    pkg_share = pathlib.Path(FindPackageShare('cube_petit_navigation').find('cube_petit_navigation'))
    args = []
    args.append(
        DeclareLaunchArgument('use_sim_time',
                              default_value='true',
                              description='Use simulation (Gazebo) clock if true'))
    args.append(
        DeclareLaunchArgument('map',
                              default_value=str(pkg_share / 'map/sample/sample.yaml'),
                              description='Full path to map yaml file to load'))
    args.append(
        DeclareLaunchArgument('keepout',
                              default_value=str(pkg_share / 'map/sample/sample_keepout.yaml'),
                              description='Full path to keepout yaml file to load'))
    args.append(
        DeclareLaunchArgument('params_file',
                              default_value=str(pkg_share / 'config/nav2_params.yaml'),
                              description='Full path to the ROS2 parameters file to use for all launched nodes'))

    args.append(
        DeclareLaunchArgument('container_name',
                              default_value='nav2_container',
                              description='the name of container that nodes will load in if use composition'))
    args.append(DeclareLaunchArgument('robot', default_value='cube_petit'))

    laser_relay = Node(package='topic_tools',
                       executable='relay',
                       name='laser_relay',
                       namespace='',
                       parameters=[
                           {
                               'input_topic': '/laser/scan'
                           },
                           {
                               'output_topic': '/cube_petit/laser/scan'
                           },
                       ],
                       output='screen')
    return LaunchDescription(args + [
        laser_relay,
        OpaqueFunction(function=launch_setup),
    ])
