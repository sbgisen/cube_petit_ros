#!/usr/bin/env python3
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
import socket

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.launch_context import LaunchContext
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace
from launch_ros.substitutions import FindPackageShare
import xacro

# from launch.actions import RegisterEventHandler
# from launch.event_handlers import OnProcessStart


def launch_in_order(context: LaunchContext, *args, **kwargs) -> list:
    ns = LaunchConfiguration('cube_petit_host_name').perform(context)

    hardware_pkg = pathlib.Path(
        FindPackageShare('cube_petit_hardware_interface').find('cube_petit_hardware_interface'))
    bringup_pkg = pathlib.Path(FindPackageShare('cube_petit_bringup').find('cube_petit_bringup'))
    speech_to_text_pkg = pathlib.Path(FindPackageShare('cube_petit_speech_to_text').find('cube_petit_speech_to_text'))
    text_to_speech_pkg = pathlib.Path(FindPackageShare('cube_petit_text_to_speech').find('cube_petit_text_to_speech'))
    face_animation_pkg = pathlib.Path(
        FindPackageShare('cube_petit_facial_animation').find('cube_petit_facial_animation'))
    shared_controller_pkg = pathlib.Path(
        FindPackageShare('cube_petit_shared_controller').find('cube_petit_shared_controller'))
    description_pkg = FindPackageShare('cube_petit_description').find('cube_petit_description')
    xacro_file = pathlib.Path(description_pkg) / 'xacro/cube_petit.xacro'
    # robot_namespaceをnsに合わせないと、URDFのroot link名がcube_petit_orange固定のまま
    # 発行され続け、navigation側のTFツリー(<robot>/base_link起点)と接続しなくなる。
    # Without matching robot_namespace to ns, the URDF's root link name would keep
    # publishing as the hardcoded cube_petit_orange, disconnecting it from the
    # navigation-side TF tree (rooted at <robot>/base_link).
    doc = xacro.process_file(xacro_file, mappings={'use_sim': 'false', 'robot_namespace': ns})
    robot_description = {'robot_description': doc.toprettyxml(indent='  ')}

    robot_state_publisher = GroupAction([
        PushRosNamespace(ns),
        Node(package='robot_state_publisher', executable='robot_state_publisher', parameters=[robot_description])
    ])

    control_node = GroupAction([
        PushRosNamespace(ns),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            str(hardware_pkg / 'launch/cube_petit_control.launch.py')),
                                 launch_arguments={
                                     'robot_namespace': ns,
                                 }.items())
    ])

    bringups = GroupAction([
        PushRosNamespace(ns),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            str(face_animation_pkg / 'launch/cube_petit_facial_animation.launch.py')),
                                 launch_arguments={
                                     'color': LaunchConfiguration('face_color'),
                                 }.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(bringup_pkg / 'launch/lidar.launch.py'))),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(bringup_pkg / 'launch/depth.launch.py'))),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(bringup_pkg / 'launch/teleop.launch.py'))),
        # 現地joystick・shared_controller(遠隔操作)・navigationの3つのcmd_velソースを
        # 優先度で調停する(twist_mux.yaml参照)。常時起動しておき、実際にどのソースも
        # 送信していなければ何も出力しない(無害)。
        # Arbitrates the three cmd_vel sources (local joystick, shared_controller remote
        # teleop, navigation) by priority (see twist_mux.yaml). Always running is harmless:
        # it simply outputs nothing while no source is actively publishing.
        GroupAction([
            PushRosNamespace('diff_drive_controller'),
            Node(
                package='twist_mux',
                executable='twist_mux',
                name='twist_mux',
                parameters=[str(bringup_pkg / 'config/twist_mux.yaml')],
                remappings=[('cmd_vel_out', 'cmd_vel')],
            ),
        ]),
        # shared_controller(共有コントローラ)のreceiver役は常時起動しておく:
        # どの機体も他機のhubから遠隔操作を受け付けられる状態にしておき、実際に
        # コントローラを物理接続してhub役になるかどうかはWeb UIから明示的に選ぶ運用
        # (2026-07-28、ありさん指示)。receiver自体は選択されていなければ何も動かさない
        # ので常時起動は無害。robot_namespaceは既に外側のPushRosNamespace(ns)で
        # 適用済みのため空文字列を渡す(二重pushを避ける)。
        # cube_petit_shared_controller's receiver role runs always-on: every individual
        # can be remote-driven by whichever robot is acting as hub, and becoming a hub
        # (physically pairing a controller) is an explicit Web UI choice instead
        # (2026-07-28). The receiver is inert while not selected, so always-on is
        # harmless. robot_namespace is left empty since the outer PushRosNamespace(ns)
        # already applies it (avoids double-pushing the namespace).
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(shared_controller_pkg / 'launch/shared_controller.launch.py')),
            launch_arguments={
                'role': 'receiver',
                'robot_namespace': '',
                # robot_namespaceは(上記の理由で)空のままだが、
                # 個体名を必要とするrobot_nameパラメータ・
                # announcement文言のYAML検索には、既にここで
                # 解決済みのnsをそのまま渡す(nodeのROBOT_NAMESPACE
                # 環境変数フォールバックはsystemdサービスの実環境に
                # 変数自体が無く効かないため、2026-07-28判明)。
                # robot_namespace stays empty (see above), but
                # robot_name (used by the node for selection
                # matching and, via this file, for the per-robot
                # announcement YAML lookup) needs the actual
                # individual name -- pass the already-resolved ns
                # directly (the node's ROBOT_NAMESPACE env var
                # fallback doesn't help here since that variable
                # isn't actually set in the systemd service's
                # environment, found 2026-07-28).
                'robot_name': ns,
            }.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            str(text_to_speech_pkg / 'launch/cube_petit_text_to_jtalk.launch.py')),
                                 launch_arguments={
                                     'voice_preset': LaunchConfiguration('voice_preset'),
                                     'voice_semitone_shift': LaunchConfiguration('voice_semitone_shift'),
                                     'voice_speed_scale': LaunchConfiguration('voice_speed_scale'),
                                 }.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(speech_to_text_pkg / 'launch/cube_petit_speech_to_text.launch.py'))),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(speech_to_text_pkg / 'launch/cube_petit_hotword_detector.launch.py'))),
        Node(
            package='cube_petit_bringup',
            executable='startup_announcer',
            name='startup_announcer',
            output='screen',
            parameters=[{
                'text': '起動しました。',
                'required_nodes': [
                    f'/{ns}/speech_action_server',
                    f'/{ns}/text_to_jtalk',
                    f'/{ns}/ldlidar_publisher_ld06',
                    f'/{ns}/controller_manager',
                ],
                'required_scan_topic': f'/{ns}/scan',
                'controller_manager_service': f'/{ns}/controller_manager',
                'required_controller_types': [
                    'joint_state_broadcaster/JointStateBroadcaster',
                    'diff_drive_controller/DiffDriveController',
                ],
                'node_check_timeout_sec': 15.0,
                'topic_check_timeout_sec': 5.0,
                'controller_check_timeout_sec': 30.0,
                'controller_grace_sec': 10.0,
                'wait_sec': 2.0,
            }],
        ),
    ])

    return [robot_state_publisher, bringups, control_node]


def generate_launch_description() -> LaunchDescription:
    """Generate launch descriptions.

    Returns:
        Launch descriptions
    """
    args = []
    hostname = socket.gethostname()
    namespace = hostname.replace('-', '_')
    args.append(DeclareLaunchArgument('cube_petit_host_name', default_value=namespace))
    # Individuals are named cube_petit_<color> (e.g. cube_petit_pink), so the
    # face color can be derived from the hostname instead of being fixed to
    # orange for every unit. Falls back to orange for hosts that don't follow
    # that naming convention (dev machines, etc.).
    face_color_default = (namespace[len('cube_petit_'):] if namespace.startswith('cube_petit_') else 'orange')
    args.append(
        DeclareLaunchArgument('face_color',
                              default_value=face_color_default,
                              description='Color for the faceBox (pink, orange, blue, green, yellow, purple)'))
    # Per-robot voice (ROSConJP conversation demo, 2026-08-04), so a listener can
    # tell orange/pink/violet apart by voice alone. Derived from the hostname the
    # same way face_color is above; only 'pink'/'violet' have a distinct preset
    # today (see cube_petit_text_to_speech VOICE_PRESETS) -- anything else
    # (orange, dev machines, ...) falls back to the unshifted 'default' voice.
    args.append(
        DeclareLaunchArgument('voice_preset',
                              default_value=face_color_default,
                              description="Per-robot voice preset name (derived from hostname, same as face_color): "
                              "'pink' (a bit higher & slower, soft), 'violet' (higher & a bit faster, playful). "
                              "Unknown names (including 'orange') fall back to the unshifted 'default' voice."))
    args.append(
        DeclareLaunchArgument('voice_semitone_shift',
                              default_value='0.0',
                              description='Extra half-tone pitch shift on top of voice_preset. '
                              '0.0 = no adjustment.'))
    args.append(
        DeclareLaunchArgument('voice_speed_scale',
                              default_value='1.0',
                              description='Extra speed multiplier on top of voice_preset. '
                              '1.0 = no adjustment.'))

    ordered_sequence = OpaqueFunction(function=launch_in_order)

    return LaunchDescription(args + [ordered_sequence])
