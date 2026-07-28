#!/usr/bin/env python

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
"""zenoh connector: bridges one CubePetit robot into Open-RMF fleet_adapter_zenoh.

This node does NOT touch rmw_zenoh_cpp / the robot's ROS graph. It opens a
plain `eclipse-zenoh` session (independent of ROS_DOMAIN_ID) and, on the ROS
side, only *wraps* interfaces that already exist:

  * pose      -> TF lookup ``map`` -> ``base_link`` (same as navigation_api_node
                 / cube_petit_python_api; ``amcl_pose`` is never published on
                 this robot, so it must not be used).
  * move/cancel/status -> the ``navigation/goal`` (std_msgs/String),
                 ``navigation/cancel`` (std_msgs/String) and ``navigation/status``
                 (std_msgs/String) topics already exposed by
                 cube_petit_navigation's navigation_api_node (see its
                 ``_on_goal`` / ``_on_cancel`` handlers). Nav2 actions are never
                 called directly.
  * speak     -> the ``speech_action_server`` action
                 (cube_petit_speech_msgs/action/Speech), same as
                 cube_petit_bringup's startup_announcer.
  * localize  -> ``geometry_msgs/PoseWithCovarianceStamped`` on the emcl2
                 particle filter's ``initialpose`` topic (AMCL-compatible
                 interface; emcl2 has no separate "localize" service).
  * battery   -> CubePetit has no battery instrumentation today: this always
                 reports a fixed dummy value. TODO(battery): replace with a
                 real reading once battery telemetry exists.
  * dock/undock -> CubePetit has no docking hardware/API in this repo today
                 (checked cube_petit_hardware_interface / cube_petit_diagnostics /
                 cube_petit_navigation: no dock sensor, no dock action/service).
                 Both always reply ``success: false``.

Run this node once per robot, namespaced to that robot (e.g. `cube_petit_orange`)
so the bare topic/action names above resolve exactly like navigation_api_node's
own topics do. `robot_name` (used to build zenoh keys) defaults to that
namespace; see the `robot_name` parameter to override it.

Requires the `eclipse-zenoh` pip package (not resolvable via rosdep -- see
package.xml / requirements.txt next to this file).
"""

from __future__ import annotations

import functools
import json
import threading
import typing

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseWithCovarianceStamped
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
import rclpy.time
from std_msgs.msg import String
from tf2_ros import Buffer
from tf2_ros import TransformListener

from cube_petit_fleet_bridge import fleet_bridge_logic as logic
from cube_petit_speech_msgs.action import Speech

try:
    import zenoh
except ImportError as _zenoh_import_error:  # pragma: no cover - exercised only without the pip dep
    zenoh = None
    _ZENOH_IMPORT_ERROR = _zenoh_import_error
else:
    _ZENOH_IMPORT_ERROR = None

_TF_TIMEOUT_SEC = 0.2
_SPEECH_SERVER_TIMEOUT_SEC = 2.0
# Speech defaults mirror cube_petit_python_api.petit.CubePetit.say()'s defaults.
_SPEECH_EMOTION = 'default'
_SPEECH_EMOTION_LEVEL = 2
_SPEECH_PITCH = 100
_SPEECH_SPEED = 100
_SPEECH_VOLUME = 30
# Covariance matches the diagonal RViz2 "2D Pose Estimate" tool publishes
# (x, y position variance 0.25 m^2, yaw variance ~0.0685 rad^2); emcl2 only
# reads the pose, not the covariance, but a well-formed message is cheap.
_INITIALPOSE_COVARIANCE = [0.0] * 36
_INITIALPOSE_COVARIANCE[0] = 0.25
_INITIALPOSE_COVARIANCE[7] = 0.25
_INITIALPOSE_COVARIANCE[35] = 0.06853892326654787


class ZenohConnector(Node):
    """Bridges one CubePetit robot's ROS graph to fleet_adapter_zenoh over plain zenoh."""

    def __init__(self) -> None:
        super().__init__('zenoh_connector')

        if zenoh is None:
            raise RuntimeError("The 'eclipse-zenoh' pip package is not installed. Install it with "
                               "`pip install --break-system-packages eclipse-zenoh` (see "
                               f'cube_petit_fleet_bridge/requirements.txt). Original error: {_ZENOH_IMPORT_ERROR}')

        # ================= parameters =================

        self.declare_parameter('robot_name', '')
        self.declare_parameter('zenoh_endpoint', 'tcp/cube-petit-orange.local:7447')
        self.declare_parameter('zenoh_mode', 'client')
        self.declare_parameter('map_name', '')
        self.declare_parameter('map_server_node', 'navigation/map_server')
        self.declare_parameter('map_lookup_timeout_sec', 2.0)
        self.declare_parameter('initialpose_topic', 'navigation/initialpose')
        self.declare_parameter('battery_level', 1.0)  # TODO(battery): real sensor, see module docstring.
        self.declare_parameter('state_publish_period_sec', 1.0)

        robot_name_param = str(self.get_parameter('robot_name').value)
        self._robot_name = robot_name_param or self.get_namespace().strip('/') or 'cube_petit'
        self.get_logger().info(f'zenoh_connector starting for robot_name={self._robot_name!r}')

        self._battery_level = float(self.get_parameter('battery_level').value)

        # 明示指定(map_nameパラメータ)があれば以後の自動更新をスキップする。無ければ
        # 起動時に一度取得し、_on_state_timerで毎回非ブロッキングに再チェックして
        # nav/SLAMの起動・終了(map_serverの出現・消失)を追従する。
        # An explicit map_name parameter disables auto-refresh entirely. Otherwise,
        # look it up once at startup and keep re-checking (non-blocking) on every
        # _on_state_timer tick, so nav/SLAM starting or stopping (map_server
        # appearing/disappearing) is picked up automatically.
        self._configured_map_name = str(self.get_parameter('map_name').value)
        self._map_name = self._configured_map_name or self._lookup_map_name_from_map_server() or 'unknown'
        self._map_name_lookup_in_flight = False
        self.get_logger().info(f'map_name={self._map_name!r}')

        # ================= TF =================

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # ================= navigation (wraps navigation_api_node's topics) =================

        self._goal_pub = self.create_publisher(String, 'navigation/goal', 10)
        self._cancel_pub = self.create_publisher(String, 'navigation/cancel', 10)
        self.create_subscription(String, 'navigation/status', self._on_navigation_status, 10)

        initialpose_topic = str(self.get_parameter('initialpose_topic').value)
        self._initialpose_pub = self.create_publisher(PoseWithCovarianceStamped, initialpose_topic, 10)

        # ================= speech =================

        self._speech_client = ActionClient(self, Speech, 'speech_action_server')

        # ================= command bookkeeping =================
        # At most one navigation/speech command in flight at a time, mirroring
        # navigation_api_node's own single-flight design.
        self._lock = threading.Lock()
        self._active: typing.Optional[dict] = None

        # ================= zenoh session =================

        self._zenoh_session = self._open_zenoh_session()
        self._pub_pose = self._zenoh_session.declare_publisher(logic.robot_key(self._robot_name, 'pose'))
        self._pub_battery = self._zenoh_session.declare_publisher(logic.robot_key(self._robot_name, 'battery'))
        self._pub_map_name = self._zenoh_session.declare_publisher(logic.robot_key(self._robot_name, 'map_name'))
        self._pub_completion = self._zenoh_session.declare_publisher(
            logic.robot_key(self._robot_name, 'command_is_completed'))
        self._sub_command = self._zenoh_session.declare_subscriber(logic.robot_key(self._robot_name, 'command'),
                                                                   self._on_zenoh_command)

        # ================= periodic state publish =================

        period = float(self.get_parameter('state_publish_period_sec').value)
        self.create_timer(period, self._on_state_timer)

        self.get_logger().info('zenoh_connector ready')

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
    # map_name discovery (best effort; falls back to the `map_name` parameter)
    # =================================================

    def _lookup_map_name_from_map_server(self) -> typing.Optional[str]:
        """Best-effort lookup of the loaded map's name from map_server's `yaml_filename` param.

        Returns:
            The derived map name, or ``None`` if the service is unavailable or
            the parameter could not be read within the configured timeout.
        """
        node_name = str(self.get_parameter('map_server_node').value).strip('/')
        service_name = f'{node_name}/get_parameters'
        timeout = float(self.get_parameter('map_lookup_timeout_sec').value)
        client = self.create_client(GetParameters, service_name)
        try:
            if not client.wait_for_service(timeout_sec=timeout):
                self.get_logger().warning(
                    f"map_server parameter service '{service_name}' not found within {timeout}s; "
                    "falling back to 'unknown'. Set the map_name parameter to silence this.")
                return None
            request = GetParameters.Request(names=['yaml_filename'])
            future = client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
            if not future.done() or future.result() is None:
                self.get_logger().warning(f"'{service_name}' did not respond within {timeout}s.")
                return None
            values = future.result().values
            if not values or values[0].type != ParameterType.PARAMETER_STRING:
                self.get_logger().warning(f"'{node_name}.yaml_filename' is unset or not a string.")
                return None
            return logic.derive_map_name_from_yaml_path(values[0].string_value)
        except Exception as error:  # noqa: BLE001 - best-effort only, must never block startup
            self.get_logger().warning(f'map_name auto-lookup failed: {error}')
            return None
        finally:
            self.destroy_client(client)

    # =================================================
    # Periodic state publish: pose / battery / map_name
    # =================================================

    def _on_state_timer(self) -> None:
        pose = self._lookup_pose()
        if pose is not None:
            x, y, yaw = pose
            self._pub_pose.put(logic.encode_pose(x, y, yaw))
        # TODO(battery): replace with a real sensor reading once available.
        self._pub_battery.put(logic.encode_battery(self._battery_level))
        self._pub_map_name.put(logic.encode_map_name(self._map_name))
        self._refresh_map_name_async()

    def _refresh_map_name_async(self) -> None:
        """Best-effort, non-blocking re-check of map_server's loaded map.

        No-op when map_name was explicitly configured. Sets map_name back to
        'unknown' if map_server has disappeared (e.g. navigation/SLAM stopped),
        so the fleet dashboard doesn't keep showing a stale map name.
        """
        if self._configured_map_name or self._map_name_lookup_in_flight:
            return
        node_name = str(self.get_parameter('map_server_node').value).strip('/')
        service_name = f'{node_name}/get_parameters'
        client = self.create_client(GetParameters, service_name)
        if not client.service_is_ready():
            self.destroy_client(client)
            if self._map_name != 'unknown':
                self.get_logger().info(f"'{service_name}' no longer available; map_name -> 'unknown'")
                self._map_name = 'unknown'
            return

        self._map_name_lookup_in_flight = True
        future = client.call_async(GetParameters.Request(names=['yaml_filename']))

        def _on_done(done_future: 'rclpy.task.Future') -> None:
            self._map_name_lookup_in_flight = False
            self.destroy_client(client)
            try:
                result = done_future.result()
            except Exception as error:  # noqa: BLE001 - best-effort only, must never raise
                self.get_logger().warning(f'map_name refresh failed: {error}')
                return
            if not result or not result.values or result.values[0].type != ParameterType.PARAMETER_STRING:
                return
            new_map_name = logic.derive_map_name_from_yaml_path(result.values[0].string_value) or 'unknown'
            if new_map_name != self._map_name:
                self.get_logger().info(f'map_name changed: {self._map_name!r} -> {new_map_name!r}')
                self._map_name = new_map_name

        future.add_done_callback(_on_done)

    def _lookup_pose(self) -> typing.Optional[typing.Tuple[float, float, float]]:
        """Look up the robot's map-frame pose from TF (map -> base_link).

        Mirrors navigation_api_node._get_current_xy() / CubePetit.where_am_i():
        this robot does not publish `amcl_pose`, so TF is the only source.
        """
        try:
            tf = self._tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time(),
                timeout=Duration(seconds=_TF_TIMEOUT_SEC),
            )
        except Exception:  # noqa: BLE001 - tf2 raises several lookup error types
            return None
        translation = tf.transform.translation
        rotation = tf.transform.rotation
        yaw = logic.yaw_from_quaternion(rotation.x, rotation.y, rotation.z, rotation.w)
        return translation.x, translation.y, yaw

    # =================================================
    # Incoming zenoh commands
    # =================================================

    def _on_zenoh_command(self, sample: 'zenoh.Sample') -> None:
        try:
            method, args, command_id = logic.parse_command(sample.payload.to_bytes())
        except logic.CommandError as error:
            self.get_logger().warning(f'Dropping malformed command: {error}')
            return

        self.get_logger().info(f'Command {command_id}: {method}({args})')

        if method == 'move_to_pose':
            self._handle_move_to_pose(args, command_id)
        elif method == 'localize':
            self._handle_localize(args, command_id)
        elif method in ('dock', 'undock'):
            self._handle_dock(method, command_id)
        elif method == 'speak':
            self._handle_speak(args, command_id)
        elif method == 'cancel_command':
            self._handle_cancel_command(command_id)
        else:
            self.get_logger().warning(f'Unsupported command method: {method!r} (supported: {logic.SUPPORTED_METHODS})')
            self._publish_completion(command_id, success=False)

    def _publish_completion(self, command_id: str, *, success: bool) -> None:
        self._pub_completion.put(logic.encode_completion(command_id, True, success))

    # ---------- move_to_pose ----------

    def _handle_move_to_pose(self, args: dict, command_id: str) -> None:
        try:
            x, y, yaw, map_name = logic.validate_move_to_pose_args(args)
        except logic.CommandError as error:
            self.get_logger().warning(str(error))
            self._publish_completion(command_id, success=False)
            return

        with self._lock:
            if self._active is not None:
                self.get_logger().warning(
                    f"Rejecting move_to_pose {command_id!r}: command {self._active['id']!r} still in flight")
                self._publish_completion(command_id, success=False)
                return
            if map_name and self._map_name != 'unknown' and map_name != self._map_name:
                # Runtime map switching is not supported: map_server's yaml_filename is fixed
                # at launch time (see cube_petit_navigation/launch/navigation.launch.py).
                self.get_logger().warning(f'move_to_pose {command_id!r}: requested map_name={map_name!r} != '
                                          f'current {self._map_name!r} (runtime map switching is not supported)')
                self._publish_completion(command_id, success=False)
                return
            self._active = {'id': command_id, 'kind': 'nav'}

        self._goal_pub.publish(String(data=logic.build_move_goal_text(x, y, yaw)))
        # Completion is reported asynchronously from _on_navigation_status().

    def _on_navigation_status(self, msg: String) -> None:
        status = msg.data
        if status not in ('arrived', 'failed'):
            return
        with self._lock:
            if self._active is None or self._active.get('kind') != 'nav':
                return
            command_id = self._active['id']
            self._active = None
        self._publish_completion(command_id, success=(status == 'arrived'))

    # ---------- localize ----------

    def _handle_localize(self, args: dict, command_id: str) -> None:
        try:
            x, y, theta, _map_name = logic.validate_localize_args(args)
        except logic.CommandError as error:
            self.get_logger().warning(str(error))
            self._publish_completion(command_id, success=False)
            return

        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        qx, qy, qz, qw = logic.quaternion_from_yaw(theta)
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        msg.pose.covariance = _INITIALPOSE_COVARIANCE
        self._initialpose_pub.publish(msg)

        # emcl2 (this robot's localizer) exposes no convergence feedback, unlike
        # navigation status. Report success as soon as the pose has been sent;
        # fleet_adapter_zenoh should treat this as "accepted", not "converged".
        self._publish_completion(command_id, success=True)

    # ---------- dock / undock ----------

    def _handle_dock(self, method: str, command_id: str) -> None:
        self.get_logger().info(
            f'{method} requested but CubePetit has no docking hardware/API in this repo today; reporting failure.')
        self._publish_completion(command_id, success=False)

    # ---------- speak ----------

    def _handle_speak(self, args: dict, command_id: str) -> None:
        try:
            text = logic.validate_speak_args(args)
        except logic.CommandError as error:
            self.get_logger().warning(str(error))
            self._publish_completion(command_id, success=False)
            return

        with self._lock:
            if self._active is not None:
                self.get_logger().warning(
                    f"Rejecting speak {command_id!r}: command {self._active['id']!r} still in flight")
                self._publish_completion(command_id, success=False)
                return
            self._active = {'id': command_id, 'kind': 'speak', 'goal_handle': None}

        if not self._speech_client.wait_for_server(timeout_sec=_SPEECH_SERVER_TIMEOUT_SEC):
            self.get_logger().warning('speech_action_server not available')
            with self._lock:
                if self._active is not None and self._active['id'] == command_id:
                    self._active = None
            self._publish_completion(command_id, success=False)
            return

        goal = Speech.Goal()
        goal.text = text
        goal.emotion = _SPEECH_EMOTION
        goal.emotion_level = _SPEECH_EMOTION_LEVEL
        goal.pitch = _SPEECH_PITCH
        goal.speed = _SPEECH_SPEED
        goal.volume = _SPEECH_VOLUME

        send_future = self._speech_client.send_goal_async(goal)
        send_future.add_done_callback(functools.partial(self._on_speech_goal_response, command_id))

    def _on_speech_goal_response(self, command_id: str, future: 'rclpy.task.Future') -> None:
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().warning(f'speak {command_id!r} rejected by speech_action_server')
            with self._lock:
                if self._active is not None and self._active['id'] == command_id:
                    self._active = None
            self._publish_completion(command_id, success=False)
            return

        with self._lock:
            if self._active is not None and self._active['id'] == command_id:
                self._active['goal_handle'] = goal_handle

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(functools.partial(self._on_speech_result, command_id))

    def _on_speech_result(self, command_id: str, future: 'rclpy.task.Future') -> None:
        result = future.result()
        success = result is not None and result.status == GoalStatus.STATUS_SUCCEEDED
        with self._lock:
            if self._active is not None and self._active['id'] == command_id:
                self._active = None
        self._publish_completion(command_id, success=success)

    # ---------- cancel_command ----------

    def _handle_cancel_command(self, command_id: str) -> None:
        with self._lock:
            active = self._active

        if active is None:
            self.get_logger().info('cancel_command received but no command is in flight')
            self._publish_completion(command_id, success=True)
            return

        if active['kind'] == 'nav':
            self._cancel_pub.publish(String(data='cancel'))
        elif active['kind'] == 'speak' and active.get('goal_handle') is not None:
            active['goal_handle'].cancel_goal_async()

        # The original command's own command_is_completed (for active['id']) is
        # still published separately once navigation/status reflects the
        # cancellation, or the speech result future resolves as canceled.
        self._publish_completion(command_id, success=True)


def main() -> None:
    rclpy.init()
    node = ZenohConnector()
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
