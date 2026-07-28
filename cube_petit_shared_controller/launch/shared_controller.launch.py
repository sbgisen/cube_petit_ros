#!/usr/bin/env python
# -*- coding:utf-8 -*-

# Copyright (c) 2026 SoftBank Corp.
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
"""Launch either the hub or the receiver half of the shared PS4 controller relay.

Usage::

    # On the hub individual (controller physically paired here, e.g. orange). This node
    # does NOT start joy_node itself -- also launch cube_petit_bringup/launch/teleop.launch.py
    # (or the full cube_petit_bringup.launch.py) so /joy exists. See this package's README.
    ros2 launch cube_petit_shared_controller shared_controller.launch.py role:=hub

    # On every controllable individual, including the hub one (e.g. orange AND pink). Do NOT
    # also launch teleop.launch.py here: this individual has no controller physically attached,
    # and running its joy_node/teleop_twist_joy_node would be pointless (no /dev/input/js0).
    ros2 launch cube_petit_shared_controller shared_controller.launch.py role:=receiver
"""

import os
import pathlib

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.actions import OpaqueFunction
from launch.launch_context import LaunchContext
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace
from launch_ros.substitutions import FindPackageShare
import yaml

_DEFAULT_SELECTED_ANNOUNCEMENT = 'コントローラ。オン。'
_DEFAULT_DESELECTED_ANNOUNCEMENT = 'コントローラ。オフ。'


def _load_announcements(config_path: str) -> dict:
    """Load announcements.yaml (see this package's config/ dir for the schema)."""
    path = pathlib.Path(config_path)
    if not path.is_file():
        return {}
    with path.open(encoding='utf-8') as config_file:
        return yaml.safe_load(config_file) or {}


def _announcement_texts(config_path: str, robot_namespace_str: str) -> tuple:
    """Resolve this individual's (selected, deselected) announcement text.

    ``off``がトップレベル(全個体共通)、個体固有の`cube_petit_<color>.off`が
    あればそちらを優先。`on`は個体ごとの指定が無ければジェネリック文言。
    ``off`` is shared across individuals unless a per-individual
    ``cube_petit_<color>.off`` override exists (which then takes priority).
    ``on`` falls back to a generic phrase when this individual isn't listed.
    """
    data = _load_announcements(config_path)
    shared_off = data.get('off', _DEFAULT_DESELECTED_ANNOUNCEMENT)
    per_robot = data.get(robot_namespace_str) or {}
    selected = per_robot.get('on', _DEFAULT_SELECTED_ANNOUNCEMENT)
    deselected = per_robot.get('off', shared_off)
    return selected, deselected


def _launch_setup(context: LaunchContext, *args, **kwargs) -> list:
    role = LaunchConfiguration('role').perform(context)
    robot_namespace = LaunchConfiguration('robot_namespace')

    if role == 'hub':
        robot_names = [
            name.strip() for name in LaunchConfiguration('robot_names').perform(context).split(',') if name.strip()
        ]
        toggle_buttons = [
            int(b.strip()) for b in LaunchConfiguration('toggle_buttons').perform(context).split(',') if b.strip()
        ]
        initial_robot_names = [
            name.strip()
            for name in LaunchConfiguration('initial_robot_names').perform(context).split(',')
            if name.strip()
        ]
        joy_topic = LaunchConfiguration('joy_topic').perform(context)
        local_cmd_vel_topic = LaunchConfiguration('local_cmd_vel_topic').perform(context)

        return [
            GroupAction([
                PushRosNamespace(robot_namespace),
                # Dedicated teleop_twist_joy_node instance, reusing the same PS4 mapping as
                # teleop.launch.py's own instance, but publishing to a *different* topic
                # (local_cmd_vel_topic) so this individual's controller_receiver_node (also
                # running here, see role:=receiver above) never feeds this relay's own
                # subscription back into itself. Does not start its own joy_node: it reads
                # the /joy topic already published by teleop.launch.py.
                #
                # ps4.config.yaml自体は使わない: そのファイルは`/**/teleop_twist_joy_node:`
                # というワイルドカードキーで、"ノード名がteleop_twist_joy_nodeである"ことが
                # 適用条件になっている。このノードはteleop.launch.py側の同名インスタンスと
                # 衝突しないよう名前を変えているため、ワイルドカードが一致せずデフォルト値
                # (enable_button等)にフォールバックしてしまい、実機で「ボタン選択は効くのに
                # スティックで動かない」不具合になっていた(2026-07-28判明)。ノード名に
                # 依存しないよう、必要な値をそのままここで指定する。
                #
                # ps4.config.yaml itself is NOT used here: it keys its parameters under the
                # wildcard `/**/teleop_twist_joy_node:`, which only applies when the node's
                # NAME is literally `teleop_twist_joy_node`. This node is deliberately renamed
                # to avoid clashing with teleop.launch.py's own instance, so the wildcard never
                # matched and it silently fell back to library defaults (wrong enable_button
                # etc.) -- on real hardware this looked like "robot selection works but the
                # stick does nothing" (found 2026-07-28). Inline the needed values instead so
                # they don't depend on the node's name.
                Node(
                    package='teleop_twist_joy',
                    executable='teleop_node',
                    name='shared_controller_teleop_twist_joy_node',
                    parameters=[{
                        'publish_stamped_twist': False,
                        'axis_linear.x': 1,
                        'scale_linear.x': 0.3,
                        'scale_linear_turbo.x': 0.7,
                        'axis_angular.yaw': 0,
                        'scale_angular.yaw': 5.0,
                        'enable_button': 0,
                        'enable_turbo_button': 5,
                    }],
                    remappings=[('joy', joy_topic), ('cmd_vel', local_cmd_vel_topic)],
                ),
                Node(
                    package='cube_petit_shared_controller',
                    executable='controller_hub_node',
                    name='controller_hub_node',
                    output='screen',
                    parameters=[{
                        'robot_names': robot_names,
                        'toggle_buttons': toggle_buttons,
                        'exclusive_modifier_button': LaunchConfiguration('exclusive_modifier_button'),
                        'required_modifier_button': LaunchConfiguration('required_modifier_button'),
                        'joy_topic': joy_topic,
                        'local_cmd_vel_topic': local_cmd_vel_topic,
                        'zenoh_endpoint': LaunchConfiguration('zenoh_router_endpoint'),
                        'zenoh_mode': LaunchConfiguration('zenoh_mode'),
                        # 空リストをROS2パラメータとして渡すと要素型を推論できずlaunchが
                        # "Expected 'value' to be one of [...], but got '()' of type
                        # 'tuple'"で落ちる(実機で確認)。デフォルト(誰も選択なし)の
                        # ときはキー自体を渡さず、ノード側のdeclare_parameterの
                        # 型付きデフォルト([])に任せる。
                        # Passing an empty list as a ROS2 parameter makes launch fail with
                        # "Expected 'value' to be one of [...], but got '()' of type
                        # 'tuple'" (confirmed on real hardware) since the element type can't
                        # be inferred. When nothing is pre-selected, omit the key entirely and
                        # let the node's own typed declare_parameter default ([]) apply.
                        **({
                            'initial_robot_names': initial_robot_names
                        } if initial_robot_names else {}),
                    }],
                ),
            ]),
        ]

    if role == 'receiver':
        # announcement_selected_text/deselected_textが未指定(空文字列)なら、
        # announcements_config(YAML)からrobot_namespace別の文言を読み込む。
        # 明示指定があればそちらを優先する。
        # When announcement_selected_text/deselected_text are left unset (empty
        # string), load per-robot_namespace text from announcements_config
        # (YAML). An explicit value always takes priority.
        #
        # bringup統合(常時receiver起動)後はrobot_namespace引数が''のまま渡される
        # (外側のcube_petit_bringup.launch.py側で既にPushRosNamespaceされているため、
        # 二重pushを避ける設計)。そのため robot_namespace.perform(context) だけでは
        # 個体名が分からずYAML検索が常に空振りし、全機体でジェネリックな
        # 「コントローラ。オン。」に固定されてしまっていた(2026-07-28判明)。
        # robot_nameパラメータと同じ ROBOT_NAMESPACE 環境変数フォールバックを適用する。
        # After the always-on bringup integration, robot_namespace is passed as '' (the
        # outer cube_petit_bringup.launch.py already applies PushRosNamespace, so this
        # avoids double-pushing). That meant robot_namespace.perform(context) alone could
        # never identify the individual, so the per-robot YAML lookup always missed and
        # every individual fell back to the generic "コントローラ。オン。" (found on real
        # hardware, 2026-07-28). Apply the same ROBOT_NAMESPACE env var fallback already
        # used for the robot_name parameter.
        robot_namespace_str = robot_namespace.perform(context) or os.environ.get('ROBOT_NAMESPACE', '')
        config_path = LaunchConfiguration('announcements_config').perform(context)
        default_selected, default_deselected = _announcement_texts(config_path, robot_namespace_str)

        selected_text = LaunchConfiguration('announcement_selected_text').perform(context) or default_selected
        deselected_text = LaunchConfiguration('announcement_deselected_text').perform(context) or default_deselected

        return [
            GroupAction([
                PushRosNamespace(robot_namespace),
                Node(
                    package='cube_petit_shared_controller',
                    executable='controller_receiver_node',
                    name='controller_receiver_node',
                    output='screen',
                    parameters=[{
                        'robot_name': LaunchConfiguration('robot_name'),
                        'output_cmd_vel_topic': LaunchConfiguration('output_cmd_vel_topic'),
                        'zenoh_endpoint': LaunchConfiguration('zenoh_router_endpoint'),
                        'zenoh_mode': LaunchConfiguration('zenoh_mode'),
                        'announcement_enabled': LaunchConfiguration('announcement_enabled'),
                        'announcement_selected_text': selected_text,
                        'announcement_deselected_text': deselected_text,
                    }],
                ),
            ]),
        ]

    raise RuntimeError(f"Unknown role {role!r}: expected 'hub' or 'receiver'")


def generate_launch_description() -> LaunchDescription:
    """Generate launch descriptions.

    Returns:
        Launch descriptions
    """
    args = [
        DeclareLaunchArgument('role',
                              default_value='hub',
                              description="Which half to launch: 'hub' (controller is physically paired here) "
                              "or 'receiver' (this individual is remote-controllable, hub included)."),
        DeclareLaunchArgument('robot_namespace',
                              default_value='cube_petit_orange',
                              description='ROS namespace of this individual (matches cube_petit_bringup.launch.py'
                              "'s cube_petit_host_name, e.g. cube_petit_orange / cube_petit_pink)."),
        # ---- role:=hub only ----
        DeclareLaunchArgument('robot_names',
                              default_value='cube_petit_orange,cube_petit_pink',
                              description='Comma-separated candidate robot names, in the same order as '
                              'toggle_buttons (robot_names[i] <-> toggle_buttons[i]).'),
        DeclareLaunchArgument('toggle_buttons',
                              default_value='2,1,3',
                              description='Comma-separated sensor_msgs/Joy buttons[] indices, one per '
                              'robot_names entry (nominally Triangle/Circle/Square on a PS4 pad). A rising '
                              "edge toggles that robot's membership in the selected set (multiple robots can "
                              'be selected at once -- they all move together from the one controller). '
                              'Buttons 0 (X, enable_button) and 5 (L1, enable_turbo_button) are already used '
                              'by ps4.config.yaml -- do not reuse those. NEEDS REAL-ROBOT VERIFICATION: '
                              'confirm the actual indices with `ros2 topic echo <joy_topic>` while pressing '
                              'each intended button.'),
        DeclareLaunchArgument('exclusive_modifier_button',
                              default_value='4',
                              description='Held while pressing a toggle_buttons entry -> switch to '
                              'controlling just that one robot (drops every other selection) instead of '
                              'adding/removing it from the group. NEEDS REAL-ROBOT VERIFICATION like '
                              'toggle_buttons.'),
        DeclareLaunchArgument('required_modifier_button',
                              default_value='11',
                              description='Joy buttons[] index that must be held for toggle_buttons to take '
                              'effect at all (D-pad up) -- prevents accidental robot-selection changes from '
                              'a stray button press. Confirmed on real hardware (2026-07-28, orange, PS4/DualShock 4 '
                              'pad): D-pad up is button 11 in this environment, not a hat axis.'),
        DeclareLaunchArgument('joy_topic',
                              default_value='diff_drive_controller/joy',
                              description='Local /joy topic (relative to robot_namespace) published by '
                              "teleop.launch.py's joy_node. Must match teleop.launch.py's own namespacing."),
        DeclareLaunchArgument('local_cmd_vel_topic',
                              default_value='diff_drive_controller/shared_controller/local_cmd_vel',
                              description='Internal-only topic the dedicated teleop_twist_joy_node instance '
                              'publishes to. Deliberately distinct from diff_drive_controller/cmd_vel; see '
                              "this file's module docstring."),
        DeclareLaunchArgument('initial_robot_names',
                              default_value='',
                              description='Comma-separated robots selected at startup, before any button '
                              'press. Empty means nobody selected (nothing moves until a button is pressed).'),
        # ---- role:=receiver only ----
        DeclareLaunchArgument('robot_name',
                              default_value='',
                              description="This individual's name, compared against controller/selected_robot. "
                              'Empty means: use the ROBOT_NAMESPACE env var if set, else robot_namespace.'),
        DeclareLaunchArgument('output_cmd_vel_topic',
                              default_value='diff_drive_controller/twist_mux/cmd_vel_shared_controller',
                              description='Topic (relative to robot_namespace) to publish TwistStamped to '
                              'while selected. Feeds into twist_mux for arbitration against navigation and '
                              'the local joystick teleop -- not diff_drive_controller/cmd_vel directly '
                              '(see cube_petit_bringup/config/twist_mux.yaml).'),
        DeclareLaunchArgument('announcement_enabled',
                              default_value='true',
                              description='Whether to speak an announcement when this individual is selected/'
                              'deselected as a controlled robot.'),
        DeclareLaunchArgument(
            'announcements_config',
            default_value=str(
                pathlib.Path(FindPackageShare('cube_petit_shared_controller').find('cube_petit_shared_controller')) /
                'config/announcements.yaml'),
            description='Path to a YAML file mapping robot_namespace -> {on, off} announcement '
            "text, plus a top-level `off` shared by every individual (see this package's "
            'config/announcements.yaml). Used only when announcement_selected_text/'
            'announcement_deselected_text are left empty.'),
        DeclareLaunchArgument('announcement_selected_text',
                              default_value='',
                              description='Text spoken via speech_action_server when selected. Empty means: '
                              'look up robot_namespace in announcements_config.'),
        DeclareLaunchArgument('announcement_deselected_text',
                              default_value='',
                              description='Text spoken via speech_action_server when deselected. Empty means: '
                              'look up robot_namespace (or the shared `off`) in announcements_config.'),
        # ---- shared ----
        DeclareLaunchArgument('zenoh_router_endpoint',
                              default_value='tcp/cube-petit-orange.local:7447',
                              description='zenoh router endpoint (router assumed to run on orange, same '
                              'default as cube_petit_fleet_bridge).'),
        DeclareLaunchArgument('zenoh_mode',
                              default_value='client',
                              description="zenoh session mode: 'client' (connect out to the router) or 'peer'."),
    ]

    return LaunchDescription(args + [OpaqueFunction(function=_launch_setup)])
