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
"""Receiver side: drives this individual's diff_drive_controller only while it is selected.

Runs on *every* individual that can be remote-controlled through the shared PS4 controller
(including the hub individual itself, e.g. ``cube_petit_orange`` -- the hub only relays the
joystick, it does not drive its own wheels directly; see controller_hub_node.py). Subscribes
the ``controller/selected_robot`` and ``controller/cmd_vel`` zenoh keys published by
ControllerHubNode and republishes velocity commands to this individual's own
``diff_drive_controller/cmd_vel`` (``geometry_msgs/msg/TwistStamped``, matching
``teleop.launch.py``'s ``publish_stamped_twist: true``) **only** while this individual's name
is a member of the selected *set*. Individuals that are not selected never move; every
individual that is selected drives the exact same command (multiple robots can be selected
at once, all moving together from the one controller).

Requires the `eclipse-zenoh` pip package, same as cube_petit_fleet_bridge (see this package's
requirements.txt).
"""

from __future__ import annotations

import json
import os
import threading

from geometry_msgs.msg import TwistStamped
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from cube_petit_shared_controller import shared_controller_logic as logic
from cube_petit_speech_msgs.action import Speech

try:
    import zenoh
except ImportError as _zenoh_import_error:  # pragma: no cover - exercised only without the pip dep
    zenoh = None
    _ZENOH_IMPORT_ERROR = _zenoh_import_error
else:
    _ZENOH_IMPORT_ERROR = None

_SPEECH_SERVER_TIMEOUT_SEC = 2.0


class ControllerReceiverNode(Node):
    """Drives the local diff_drive_controller only while this robot is the selected one."""

    def __init__(self) -> None:
        super().__init__('controller_receiver_node')

        if zenoh is None:
            raise RuntimeError(
                "The 'eclipse-zenoh' pip package is not installed. Install it with "
                '`pip install --break-system-packages eclipse-zenoh` (see '
                f'cube_petit_shared_controller/requirements.txt). Original error: {_ZENOH_IMPORT_ERROR}')

        # ================= parameters =================

        self.declare_parameter('robot_name', '')
        # diff_drive_controller/cmd_velへ直接出さずtwist_mux用の入力トピックへ出す(優先度90、
        # navigationより上・現地joystickより下。twist_mux.yaml参照)。
        # Publishes to twist_mux's input topic rather than diff_drive_controller/cmd_vel
        # directly, so twist_mux can arbitrate against navigation/local joystick (priority 90;
        # see twist_mux.yaml).
        self.declare_parameter('output_cmd_vel_topic', 'diff_drive_controller/twist_mux/cmd_vel_shared_controller')
        self.declare_parameter('zenoh_endpoint', 'tcp/cube-petit-orange.local:7447')
        self.declare_parameter('zenoh_mode', 'client')
        self.declare_parameter('announcement_enabled', True)
        self.declare_parameter('announcement_selected_text', 'コントローラオン!')
        self.declare_parameter('announcement_deselected_text', 'コントローラオフ!')
        self.declare_parameter('announcement_emotion', 'happiness')
        self.declare_parameter('announcement_emotion_level', 2)
        self.declare_parameter('announcement_pitch', 120)
        self.declare_parameter('announcement_speed', 100)
        self.declare_parameter('announcement_volume', 80)

        robot_name_param = str(self.get_parameter('robot_name').value)
        env_robot_name = os.environ.get('ROBOT_NAMESPACE')
        self._robot_name = logic.resolve_robot_name(robot_name_param, env_robot_name, self.get_namespace(),
                                                    'cube_petit')
        self.get_logger().info(f'controller_receiver_node starting for robot_name={self._robot_name!r}')

        self._announcement_enabled = bool(self.get_parameter('announcement_enabled').value)

        # ================= ROS I/O =================

        output_topic = str(self.get_parameter('output_cmd_vel_topic').value)
        self._cmd_vel_pub = self.create_publisher(TwistStamped, output_topic, 10)
        self._speech_client = ActionClient(self, Speech, 'speech_action_server')

        # ================= selection state =================
        # Mutated from zenoh's background subscriber thread (see _on_selected_robot /
        # _on_cmd_vel below) and read from there too; rclpy publishers/ActionClients are
        # themselves safe to call from a non-rclpy thread (cube_petit_fleet_bridge's
        # zenoh_connector.py does the same), but this flag needs its own lock since it is
        # both read and written from that thread.
        self._lock = threading.Lock()
        self._is_selected = False

        # ================= zenoh session =================

        self._zenoh_session = self._open_zenoh_session()
        self._sub_selected_robot = self._zenoh_session.declare_subscriber(logic.SELECTED_ROBOT_KEY,
                                                                          self._on_selected_robot)
        self._sub_cmd_vel = self._zenoh_session.declare_subscriber(logic.CMD_VEL_KEY, self._on_cmd_vel)

        self.get_logger().info('controller_receiver_node ready')

    def close(self) -> None:
        """Release the zenoh session. Safe to call multiple times."""
        session = getattr(self, '_zenoh_session', None)
        if session is not None and not session.is_closed():
            session.close()

    # =================================================
    # zenoh session setup
    # =================================================

    def _open_zenoh_session(self) -> 'zenoh.Session':
        endpoint = str(self.get_parameter('zenoh_endpoint').value)
        mode = str(self.get_parameter('zenoh_mode').value)
        config = zenoh.Config()
        config.insert_json5('mode', json.dumps(mode))
        config.insert_json5('connect/endpoints', json.dumps([endpoint]))
        self.get_logger().info(f'Opening zenoh session: mode={mode!r} connect={endpoint!r}')
        return zenoh.open(config)

    # =================================================
    # controller/selected_robot -> is this individual selected?
    # =================================================

    def _on_selected_robot(self, sample: 'zenoh.Sample') -> None:
        try:
            selected_robot_names = logic.decode_selected_robots(sample.payload.to_bytes())
        except logic.ControllerMessageError as error:
            self.get_logger().warning(f'Dropping malformed controller/selected_robot payload: {error}')
            return

        newly_selected = self._robot_name in selected_robot_names
        with self._lock:
            was_selected = self._is_selected
            self._is_selected = newly_selected

        if newly_selected and not was_selected:
            self.get_logger().info(f'Selected as one of the controlled robots (selection={selected_robot_names!r})')
            self._announce(str(self.get_parameter('announcement_selected_text').value))
        elif was_selected and not newly_selected:
            self.get_logger().info(f'No longer selected (selection={selected_robot_names!r}); publishing zero cmd_vel')
            self._publish_cmd_vel(0.0, 0.0)
            self._announce(str(self.get_parameter('announcement_deselected_text').value))

    # =================================================
    # controller/cmd_vel -> drive the local diff_drive_controller, only if selected
    # =================================================

    def _on_cmd_vel(self, sample: 'zenoh.Sample') -> None:
        try:
            linear_x, angular_z = logic.decode_cmd_vel(sample.payload.to_bytes())
        except logic.ControllerMessageError as error:
            self.get_logger().warning(f'Dropping malformed controller/cmd_vel payload: {error}')
            return

        with self._lock:
            is_selected = self._is_selected
        if not is_selected:
            return
        self._publish_cmd_vel(linear_x, angular_z)

    def _publish_cmd_vel(self, linear_x: float, angular_z: float) -> None:
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        # frame_id intentionally left empty: diff_drive_controller's cmd_vel input does not
        # use it, matching teleop_twist_joy_node's own publish_stamped_twist output.
        msg.twist.linear.x = linear_x
        msg.twist.angular.z = angular_z
        self._cmd_vel_pub.publish(msg)

    # =================================================
    # "Controller on/off!" announcement on selection change
    # =================================================

    def _announce(self, text: str) -> None:
        if not self._announcement_enabled or not text:
            return

        if not self._speech_client.wait_for_server(timeout_sec=_SPEECH_SERVER_TIMEOUT_SEC):
            self.get_logger().warning(f'speech_action_server not available; skipping announcement {text!r}')
            return

        goal = Speech.Goal()
        goal.text = text
        goal.emotion = str(self.get_parameter('announcement_emotion').value)
        goal.emotion_level = int(self.get_parameter('announcement_emotion_level').value)
        goal.pitch = int(self.get_parameter('announcement_pitch').value)
        goal.speed = int(self.get_parameter('announcement_speed').value)
        goal.volume = int(self.get_parameter('announcement_volume').value)

        # Fire-and-forget: this is a cosmetic announcement, not a tracked command like
        # cube_petit_fleet_bridge's `speak`, so no command_is_completed-style bookkeeping.
        self._speech_client.send_goal_async(goal)


def main() -> None:
    rclpy.init()
    node = ControllerReceiverNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
