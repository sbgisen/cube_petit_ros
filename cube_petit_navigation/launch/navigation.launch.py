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
import socket

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
from nav2_common.launch import ReplaceString
from nav2_common.launch import RewrittenYaml


def _default_robot_namespace() -> str:
    """hostname(cube-petit-<color>)からROS名前空間を導く / Derive the ROS namespace from hostname.

    cube_petit_web_interface.helpers.resolve_namespace()と同じ発想(PR #102参照)。
    以前はnavigation_orange.launch.py/navigation_pink.launch.py等の機体別ラッパーで
    固定値を渡していたが、hostnameから自動導出することで機体ごとのラッパーファイルを
    廃止できる(2026-07-27)。
    Same idea as cube_petit_web_interface.helpers.resolve_namespace() (see PR #102).
    Previously per-robot wrapper launch files (navigation_orange.launch.py,
    navigation_pink.launch.py, ...) hardcoded this value; auto-deriving it from hostname
    lets us drop those wrapper files entirely.
    """
    namespace = socket.gethostname().replace('-', '_')
    return namespace if namespace.startswith('cube_petit_') else 'cube_petit_orange'


def launch_setup(context: LaunchContext, *args, **kwargs) -> list:
    actions = []
    map_file = LaunchConfiguration('map').perform(context)
    keepout_file = LaunchConfiguration('keepout').perform(context)

    lifecycle_nodes = [
        'controller_server',
        'smoother_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower',
    ]
    filter_nodes = [
        'keepout_mask_server',
        'costmap_filter_info_server',
    ]
    autostart = LaunchConfiguration('autostart', default='true')
    # Replace the `<robot>` placeholder in the params file with the actual robot name so that
    # a single parameter file can serve any robot individual.
    params_file = ReplaceString(source_file=LaunchConfiguration('params_file'),
                                replacements={'<robot>': LaunchConfiguration('robot')})
    param_substitutions = {
        'autostart': autostart,
        'filter_info_topic': ['/', LaunchConfiguration('robot'), '/navigation/costmap_filter_info'],
    }

    configured_params = RewrittenYaml(source_file=params_file,
                                      root_key=[LaunchConfiguration('robot'), '/navigation'],
                                      param_rewrites=param_substitutions,
                                      convert_types=False)

    actions = [
        PushRosNamespace([LaunchConfiguration('robot'), '/navigation']),
        SetParameter('use_sim_time', LaunchConfiguration('use_sim_time')),
        SetParametersFromFile(configured_params),
        SetRemap('/scan', ['/', LaunchConfiguration('robot'), '/scan']),
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
                                   ('scan', ['/', LaunchConfiguration('robot'), '/scan']),
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
                ComposableNode(package='nav2_map_server',
                               plugin='nav2_map_server::MapServer',
                               name='keepout_mask_server',
                               parameters=[{
                                   'yaml_filename': keepout_file
                               }]),
                ComposableNode(package='nav2_map_server',
                               plugin='nav2_map_server::CostmapFilterInfoServer',
                               name='costmap_filter_info_server',
                               parameters=[{
                                   'mask_topic': ['/', LaunchConfiguration('robot'), '/navigation/keepout_mask']
                               }]),
                ComposableNode(package='nav2_lifecycle_manager',
                               plugin='nav2_lifecycle_manager::LifecycleManager',
                               name='lifecycle_manager_filters',
                               parameters=[{
                                   'autostart': autostart,
                                   'node_names': filter_nodes
                               }]),
                ComposableNode(package='emcl2',
                               plugin='emcl2::EMcl2Node',
                               name='emcl',
                               parameters=[params_file],
                               remappings=[('scan', ['/', LaunchConfiguration('robot'), '/scan'])]),
                ComposableNode(package='nav2_lifecycle_manager',
                               plugin='nav2_lifecycle_manager::LifecycleManager',
                               name='lifecycle_manager_localization',
                               parameters=[{
                                   'autostart': autostart,
                                   'node_names': ['map_server', 'emcl']
                               }]),
            ]),
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
                              default_value='false',
                              description='Use simulation (Gazebo) clock if true'))
    args.append(
        DeclareLaunchArgument('map',
                              default_value=str(pkg_share / 'map/test/test.yaml'),
                              description='Full path to map yaml file to load'))
    args.append(
        DeclareLaunchArgument('keepout',
                              default_value=str(pkg_share / 'map/test/test_keepout.yaml'),
                              description='Full path to keepout yaml file to load'))
    args.append(
        DeclareLaunchArgument('params_file',
                              default_value=str(pkg_share / 'config/nav2_params.yaml'),
                              description='Full path to the ROS2 parameters file to use for all launched nodes'))

    args.append(
        DeclareLaunchArgument('container_name',
                              default_value='nav2_container',
                              description='the name of container that nodes will load in if use composition'))
    args.append(
        DeclareLaunchArgument('robot',
                              default_value=_default_robot_namespace(),
                              description='Robot namespace (hostnameから自動導出、明示指定で上書き可能 / '
                              'auto-derived from hostname, override explicitly if needed).'))

    laser_relay = Node(package='topic_tools',
                       executable='relay',
                       name='laser_relay',
                       namespace='',
                       parameters=[
                           {
                               'input_topic': '/laser/scan'
                           },
                           {
                               'output_topic': ['/', LaunchConfiguration('robot'), '/laser/scan']
                           },
                       ],
                       output='screen')

    bridge_base_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_bridge',
        arguments=[
            '0',
            '0',
            '0',
            '0',
            '0',
            '0',
            [LaunchConfiguration('robot'), '/base_link'],
            'base_footprint',
        ],
        output='screen',
    )
    bridge_base_link2 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_bridge',
        arguments=[
            '0',
            '0',
            '0',
            '0',
            '0',
            '0',
            'base_footprint',
            'base_link',
        ],
        output='screen',
    )
    # twist_muxが調停できるよう、diff_drive_controller/cmd_velへ直接relayせずtwist_mux用の
    # 入力トピックへ出す(twist_mux.yamlのnavigationエントリ、優先度10=最低。joystickや
    # shared_controllerが動いていない間だけ有効になる)。
    # Relay into twist_mux's input topic instead of diff_drive_controller/cmd_vel directly, so
    # twist_mux can arbitrate (see twist_mux.yaml's `navigation` entry, priority 10 = lowest;
    # only takes effect while neither joystick nor shared_controller is active).
    relay = Node(
        package='topic_tools',
        executable='relay',
        arguments=[
            ['/', LaunchConfiguration('robot'), '/navigation/cmd_vel'],
            ['/', LaunchConfiguration('robot'), '/diff_drive_controller/twist_mux/cmd_vel_nav'],
        ],
        output='screen',
    )

    return LaunchDescription(args + [
        laser_relay,
        bridge_base_link,
        bridge_base_link2,
        relay,
        OpaqueFunction(function=launch_setup),
    ])
