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
"""Hub-side relay: one PS4 controller, several robots, all driven by the same stick at once.

Runs on the individual the controller is physically (Bluetoothed) paired to -- by default
``cube_petit_orange``. Does **not** reimplement joystick-to-Twist math: this node only

  * watches the local ``/joy`` topic (``sensor_msgs/msg/Joy``, published by the existing
    ``joy_node`` started by ``cube_petit_bringup/launch/teleop.launch.py``) to detect the
    rising edge of each ``toggle_buttons[i]`` (one dedicated button per ``robot_names[i]``)
    and toggle that robot's membership in the *selected set*, publishing the whole set to the
    ``controller/selected_robot`` zenoh key; and
  * relays the *local* velocity command computed by a second, dedicated
    ``teleop_twist_joy_node`` instance (started by this package's launch file, reusing the
    same ``ps4.config.yaml``) into the ``controller/cmd_vel`` zenoh key.

Every individual whose name is currently in the selected set drives the exact same
``cmd_vel`` (see ``controller_receiver_node.py``'s membership check) -- pressing more than
one robot's toggle button makes them all move together from this one controller, e.g. for
"send everyone forward" style demos. Press a robot's button again to drop it back out.

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
        # defaults to [2, 1, 3] (nominally Triangle/Circle/Square, one per robot_names entry
        # in order) precisely to avoid colliding with those. Confirm with
        # `ros2 topic echo <joy_topic>` while pressing each intended button. Must be the same
        # length as robot_names -- toggle_buttons[i] toggles robot_names[i]'s membership in
        # the selected set.
        self.declare_parameter('toggle_buttons', [2, 1, 3])
        # Held while pressing a toggle_buttons entry -> "switch to controlling just this one"
        # (exclusive_robot_selection) instead of the default add/remove-from-the-group
        # behavior (toggle_robot_selection). NEEDS REAL-ROBOT VERIFICATION like toggle_buttons.
        self.declare_parameter('exclusive_modifier_button', 4)
        # 選択トグル自体を有効にするためのガード: これ(D-pad上、デフォルトaxes[7])を押して
        # いない間はtoggle_buttonsの立ち上がりを無視する(誤操作でロボット選択が
        # 変わらないようにするための安全策)。NEEDS REAL-ROBOT VERIFICATION: D-padが
        # axesのhat軸として現れるかbuttonsとして現れるかはドライバ依存。
        # Guard that must be held for toggle_buttons to take effect at all (D-pad up,
        # defaults to axes[7]) -- prevents accidental robot-selection changes from a
        # stray button press. NEEDS REAL-ROBOT VERIFICATION: whether the D-pad shows up
        # as a hat axis or as buttons is driver-dependent.
        self.declare_parameter('required_modifier_axis', 7)
        self.declare_parameter('required_modifier_axis_value', 1.0)
        self.declare_parameter('required_modifier_axis_tolerance', 0.5)
        self.declare_parameter('joy_topic', 'diff_drive_controller/joy')
        self.declare_parameter('local_cmd_vel_topic', 'diff_drive_controller/shared_controller/local_cmd_vel')
        self.declare_parameter('zenoh_endpoint', 'tcp/cube-petit-orange.local:7447')
        self.declare_parameter('zenoh_mode', 'client')
        # Comma-handled at the launch-file level; empty means "nobody selected at startup"
        # (safest default: nothing moves until a button is pressed).
        self.declare_parameter('initial_robot_names', typing.cast(typing.List[str], []))

        self._robot_names: typing.List[str] = [str(name) for name in self.get_parameter('robot_names').value]
        if not self._robot_names:
            raise RuntimeError('robot_names parameter must not be empty')
        self._toggle_buttons: typing.List[int] = [int(b) for b in self.get_parameter('toggle_buttons').value]
        if len(self._toggle_buttons) != len(self._robot_names):
            raise RuntimeError(f'toggle_buttons ({self._toggle_buttons!r}) must have the same length as '
                               f'robot_names ({self._robot_names!r})')
        self._exclusive_modifier_button = int(self.get_parameter('exclusive_modifier_button').value)
        self._required_modifier_axis = int(self.get_parameter('required_modifier_axis').value)
        self._required_modifier_axis_value = float(self.get_parameter('required_modifier_axis_value').value)
        self._required_modifier_axis_tolerance = float(self.get_parameter('required_modifier_axis_tolerance').value)

        initial_names = [str(name) for name in self.get_parameter('initial_robot_names').value]
        self._selected: typing.FrozenSet[str] = frozenset(name for name in initial_names if name in self._robot_names)

        self._previous_buttons: typing.Sequence[int] = ()

        self.get_logger().info(f'robot_names={self._robot_names} toggle_buttons={self._toggle_buttons} '
                               f'initial selection={sorted(self._selected)!r}')

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
        self._publish_selected_robots()

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
    # /joy -> selected robot set toggle (one button per robot)
    # =================================================

    def _required_modifier_held(self, msg: Joy) -> bool:
        """Whether the toggle-enable guard (D-pad up by default) is currently held."""
        axis_index = self._required_modifier_axis
        if not (0 <= axis_index < len(msg.axes)):
            return False
        return abs(msg.axes[axis_index] - self._required_modifier_axis_value) <= self._required_modifier_axis_tolerance

    def _on_joy(self, msg: Joy) -> None:
        guard_held = self._required_modifier_held(msg)
        modifier_held = (0 <= self._exclusive_modifier_button < len(msg.buttons) and
                         bool(msg.buttons[self._exclusive_modifier_button]))
        changed = False
        for robot_name, button_index in zip(self._robot_names, self._toggle_buttons):
            if not logic.button_rising_edge(self._previous_buttons, msg.buttons, button_index):
                continue
            if not guard_held:
                self.get_logger().info(f'toggle_buttons[{button_index}] pressed without the required '
                                       'modifier (D-pad up) held -- ignored')
                continue
            if modifier_held:
                self._selected = logic.exclusive_robot_selection(robot_name)
                self.get_logger().info(f'toggle_buttons[{button_index}]+modifier pressed -> '
                                       f'switched to controlling only {robot_name}')
            else:
                self._selected = logic.toggle_robot_selection(self._selected, robot_name)
                now_in = robot_name in self._selected
                state = 'selected' if now_in else 'deselected'
                self.get_logger().info(f'toggle_buttons[{button_index}] pressed -> '
                                       f'{robot_name} {state} '
                                       f'(current selection: {sorted(self._selected)!r})')
            changed = True
        if changed:
            self._publish_selected_robots()
        self._previous_buttons = msg.buttons

    def _publish_selected_robots(self) -> None:
        self._pub_selected_robot.put(logic.encode_selected_robots(sorted(self._selected)))

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
