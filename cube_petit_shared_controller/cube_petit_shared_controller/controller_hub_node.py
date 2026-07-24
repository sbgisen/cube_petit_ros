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
"""Hub-side relay: one PS4 controller, several robots, switched by a button press.

Runs on the individual the controller is physically (Bluetoothed) paired to -- by default
``cube_petit_orange``. Does **not** reimplement joystick-to-Twist math: this node only

  * watches the local ``/joy`` topic (``sensor_msgs/msg/Joy``, published by the existing
    ``joy_node`` started by ``cube_petit_bringup/launch/teleop.launch.py``) to detect the
    rising edge of ``switch_button`` and cycle through ``robot_names``, publishing the
    selection to the ``controller/selected_robot`` zenoh key; and
  * relays the *local* velocity command computed by a second, dedicated
    ``teleop_twist_joy_node`` instance (started by this package's launch file, reusing the
    same ``ps4.config.yaml``) into the ``controller/cmd_vel`` zenoh key.

That second teleop_twist_joy_node instance publishes to ``local_cmd_vel_topic`` -- a topic
distinct from the real ``diff_drive_controller/cmd_vel`` actuator topic -- specifically so this
node's subscription can never be fed back into by cube_petit_shared_controller's own receiver
node (which also runs on this same hub individual; see controller_receiver_node.py). Relaying
the actual actuator topic here would create an infinite hub->zenoh->receiver->hub loop whenever
the hub itself is the selected robot.

Requires the `eclipse-zenoh` pip package, same as cube_petit_fleet_bridge (see this package's
requirements.txt).
"""

from __future__ import annotations

import json
import typing

from geometry_msgs.msg import Twist
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Joy

from cube_petit_shared_controller import shared_controller_logic as logic

try:
    import zenoh
except ImportError as _zenoh_import_error:  # pragma: no cover - exercised only without the pip dep
    zenoh = None
    _ZENOH_IMPORT_ERROR = _zenoh_import_error
else:
    _ZENOH_IMPORT_ERROR = None


class ControllerHubNode(Node):
    """Detects the switch-robot button press and relays joystick cmd_vel over zenoh."""

    def __init__(self) -> None:
        super().__init__('controller_hub_node')

        if zenoh is None:
            raise RuntimeError(
                "The 'eclipse-zenoh' pip package is not installed. Install it with "
                '`pip install --break-system-packages eclipse-zenoh` (see '
                f'cube_petit_shared_controller/requirements.txt). Original error: {_ZENOH_IMPORT_ERROR}')

        # ================= parameters =================

        self.declare_parameter('robot_names', ['cube_petit_orange', 'cube_petit_pink'])
        # NEEDS REAL-ROBOT VERIFICATION: button indices depend on the OS/driver's joystick
        # mapping and can differ from the nominal PS4 layout. Button 0 (X) and button 5 (L1)
        # are already used by ps4.config.yaml (enable_button / enable_turbo_button); this
        # defaults to button 2 (nominally Triangle) precisely to avoid colliding with those.
        # Confirm with `ros2 topic echo <joy_topic>` while pressing the intended button.
        self.declare_parameter('switch_button', 2)
        self.declare_parameter('joy_topic', 'diff_drive_controller/joy')
        self.declare_parameter('local_cmd_vel_topic', 'diff_drive_controller/shared_controller/local_cmd_vel')
        self.declare_parameter('zenoh_endpoint', 'tcp/cube-petit-orange.local:7447')
        self.declare_parameter('zenoh_mode', 'client')
        # Empty means "start on robot_names[0]".
        self.declare_parameter('initial_robot_name', '')

        self._robot_names: typing.List[str] = [str(name) for name in self.get_parameter('robot_names').value]
        if not self._robot_names:
            raise RuntimeError('robot_names parameter must not be empty')
        self._switch_button = int(self.get_parameter('switch_button').value)

        initial_robot_name = str(self.get_parameter('initial_robot_name').value)
        if initial_robot_name and initial_robot_name in self._robot_names:
            self._selected_index = self._robot_names.index(initial_robot_name)
        else:
            self._selected_index = 0

        self._previous_buttons: typing.Sequence[int] = ()

        self.get_logger().info(f'robot_names={self._robot_names} switch_button={self._switch_button} '
                               f'initial selection={self._robot_names[self._selected_index]!r}')

        # ================= zenoh session =================

        self._zenoh_session = self._open_zenoh_session()
        self._pub_selected_robot = self._zenoh_session.declare_publisher(logic.SELECTED_ROBOT_KEY)
        self._pub_cmd_vel = self._zenoh_session.declare_publisher(logic.CMD_VEL_KEY)

        # ================= ROS I/O =================

        joy_topic = str(self.get_parameter('joy_topic').value)
        local_cmd_vel_topic = str(self.get_parameter('local_cmd_vel_topic').value)
        self.create_subscription(Joy, joy_topic, self._on_joy, 10)
        self.create_subscription(Twist, local_cmd_vel_topic, self._on_local_cmd_vel, 10)

        # Publish the initial selection immediately so every receiver picks it up on startup
        # instead of waiting for the first button press.
        self._publish_selected_robot()

        self.get_logger().info('controller_hub_node ready')

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
    # /joy -> selected robot toggle
    # =================================================

    def _on_joy(self, msg: Joy) -> None:
        if logic.button_rising_edge(self._previous_buttons, msg.buttons, self._switch_button):
            self._selected_index = logic.next_robot_index(self._selected_index, len(self._robot_names))
            selected = self._robot_names[self._selected_index]
            self.get_logger().info(f'switch_button pressed -> selected robot: {selected}')
            self._publish_selected_robot()
        self._previous_buttons = msg.buttons

    def _publish_selected_robot(self) -> None:
        selected = self._robot_names[self._selected_index]
        self._pub_selected_robot.put(logic.encode_selected_robot(selected))

    # =================================================
    # local cmd_vel -> zenoh relay
    # =================================================

    def _on_local_cmd_vel(self, msg: Twist) -> None:
        self._pub_cmd_vel.put(logic.encode_cmd_vel(msg.linear.x, msg.angular.z))


def main() -> None:
    rclpy.init()
    node = ControllerHubNode()
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
