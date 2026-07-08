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
"""CubePetit facade: control the robot from plain Python without touching rclpy.

Example:
    >>> from cube_petit_python_api import CubePetit
    >>> with CubePetit() as robot:
    ...     robot.say('こんにちは')

The facade owns a private node and spins it on a daemon background thread, so
the caller never needs ``rclpy.init()`` nor ``rclpy.spin()``. Every method is
synchronous, bounded by a timeout and raises :class:`CubePetitNotRunning`
(with a Japanese hint) instead of hanging when the robot stack is down.
"""

from __future__ import annotations

import threading
import time
import typing
import uuid

from action_msgs.msg import GoalStatus
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from tf2_ros import Buffer
from tf2_ros import TransformListener

from cube_petit_facial_animation_msgs.msg import FaceExpression
from cube_petit_navigation_msgs.srv import SavePlace
from cube_petit_python_api import petit_names
from cube_petit_python_api.exceptions import CubePetitClosed
from cube_petit_python_api.exceptions import CubePetitError
from cube_petit_python_api.exceptions import CubePetitNotRunning
from cube_petit_python_api.petit_names import RobotPose
from cube_petit_speech_msgs.action import Speech

_POLL_INTERVAL = 0.05


class CubePetit:
    """Simple synchronous API for cube_petit — no ROS knowledge required.

    The instance is safe to create and close repeatedly within one process
    (each instance owns its own rclpy context). Methods may be called from
    multiple threads; the internal executor runs on a daemon thread and stops
    when :meth:`close` is called or the instance is garbage collected.

    Args:
        robot: Robot namespace (default: env ``PETIT_ROBOT_NS`` or ``cube_petit_orange``).
        timeout: Default timeout [s] for discovering robot interfaces.
        domain_id: Optional ROS domain id. ``None`` uses ``ROS_DOMAIN_ID``.
    """

    def __init__(self,
                 robot: typing.Optional[str] = None,
                 *,
                 timeout: float = 5.0,
                 domain_id: typing.Optional[int] = None) -> None:
        """Start the private node and the background executor thread."""
        self._namespace = petit_names.resolve_robot_namespace(robot)
        self._timeout = petit_names.validate_timeout(timeout)
        self._closed = False
        self._lock = threading.RLock()

        self._context = rclpy.Context()
        self._context.init(args=None, domain_id=domain_id)
        node_name = f'cube_petit_api_{uuid.uuid4().hex[:8]}'
        self._node: Node = rclpy.create_node(node_name, context=self._context)
        self._executor = MultiThreadedExecutor(num_threads=2, context=self._context)
        self._executor.add_node(self._node)
        self._thread = threading.Thread(target=self._spin, name=node_name, daemon=True)
        self._thread.start()

        self._speech_client: typing.Optional[ActionClient] = None
        self._tf_buffer: typing.Optional[Buffer] = None
        self._tf_listener: typing.Optional[TransformListener] = None

    # =================================================
    # Lifecycle
    # =================================================

    def __enter__(self) -> 'CubePetit':
        """Return self for use as a context manager."""
        return self

    def __exit__(self, *exc_info: object) -> None:
        """Close the facade when leaving a ``with`` block."""
        self.close()

    def __del__(self) -> None:
        """Close the facade on garbage collection (best effort)."""
        try:
            self.close()
        except Exception:  # noqa: S110
            pass

    @property
    def robot(self) -> str:
        """The robot namespace this instance talks to."""  # noqa: D401
        return self._namespace

    def close(self) -> None:
        """Stop the background executor and release all ROS resources.

        Safe to call multiple times.
        """
        with self._lock:
            if self._closed:
                return
            self._closed = True
        self._executor.shutdown(timeout_sec=2.0)
        self._thread.join(timeout=5.0)
        try:
            self._node.destroy_node()
        finally:
            self._context.try_shutdown()

    # =================================================
    # Speech
    # =================================================

    def say(self,
            text: str,
            emotion: str = 'normal',
            *,
            wait: bool = True,
            timeout: typing.Optional[float] = None) -> bool:
        """Make the robot speak.

        Args:
            text: Text to speak (Japanese OK).
            emotion: One of ``normal`` / ``happy`` / ``angry`` / ``sad`` / ``shout``
                (speech server aliases such as ``default`` also work).
            wait: Wait until the speech finishes. ``False`` returns right after
                the robot accepts the request.
            timeout: Timeout [s] for reaching the robot. Defaults to the
                constructor value.

        Returns:
            True if the speech succeeded (always True when ``wait=False``).

        Raises:
            ValueError: If the arguments are invalid.
            CubePetitNotRunning: If the speech server is unreachable within the timeout.
            CubePetitError: If the request is rejected or does not finish in time.
        """
        if not isinstance(text, str) or not text.strip():
            raise ValueError('text must be a non-empty string')
        goal = Speech.Goal()
        goal.text = text
        goal.emotion = petit_names.normalize_say_emotion(emotion)
        goal.emotion_level = 2
        goal.pitch = 100
        goal.speed = 100
        goal.volume = 30
        timeout = self._resolve_timeout(timeout)

        client = self._get_speech_client()
        if not client.wait_for_server(timeout_sec=timeout):
            action_name = petit_names.speech_action_name(self._namespace)
            raise CubePetitNotRunning(f'speech action {action_name} が見つかりません')
        goal_handle = self._wait_future(client.send_goal_async(goal), timeout, '発話リクエストの送信')
        if not goal_handle.accepted:
            raise CubePetitError('発話リクエストが拒否されました。')
        if not wait:
            return True
        # Speech duration scales with text length; allow a generous bound.
        result_timeout = max(timeout, 30.0 + 0.5 * len(text))
        result = self._wait_future(goal_handle.get_result_async(), result_timeout, '発話の完了待ち')
        return result.status == GoalStatus.STATUS_SUCCEEDED

    # =================================================
    # Navigation
    # =================================================

    def move_to(self,
                target: typing.Union[str, typing.Sequence[float]],
                *,
                wait: bool = False,
                timeout: typing.Optional[float] = None,
                arrival_timeout: float = 300.0) -> bool:
        """Send a navigation goal.

        Args:
            target: ``'favorite'`` / ``'patrol'`` or a map pose ``(x, y, yaw)``.
            wait: Wait until the robot arrives (or fails).
            timeout: Timeout [s] for reaching the robot. Defaults to the constructor value.
            arrival_timeout: Timeout [s] for the arrival when ``wait=True``.

        Returns:
            True on arrival when ``wait=True``; True right after sending otherwise.

        Raises:
            ValueError: If the target cannot be interpreted.
            CubePetitNotRunning: If the navigation stack is unreachable within the timeout.
            CubePetitError: If ``wait=True`` and the robot does not arrive in time.
        """
        goal_text = petit_names.build_move_goal(target)
        timeout = self._resolve_timeout(timeout)
        arrival_timeout = petit_names.validate_timeout(arrival_timeout, 'arrival_timeout')
        self._ensure_open()

        publisher = self._node.create_publisher(String, petit_names.navigation_goal_topic(self._namespace), 10)
        events: 'typing.List[str]' = []
        arrived = threading.Event()

        def _on_status(msg: String) -> None:
            events.append(msg.data)
            if msg.data in ('arrived', 'failed'):
                arrived.set()

        subscription = None
        try:
            self._wait_for_subscriber(publisher, timeout, petit_names.navigation_goal_topic(self._namespace))
            if wait:
                subscription = self._node.create_subscription(String,
                                                              petit_names.navigation_status_topic(self._namespace),
                                                              _on_status, 10)
            publisher.publish(String(data=goal_text))
            if not wait:
                return True
            if not arrived.wait(arrival_timeout):
                raise CubePetitError(f'{arrival_timeout:.0f}秒待っても移動が完了しませんでした。')
            return events[-1] == 'arrived'
        finally:
            if subscription is not None:
                self._node.destroy_subscription(subscription)
            self._node.destroy_publisher(publisher)

    def cancel_move(self, *, timeout: typing.Optional[float] = None) -> None:
        """Cancel the current navigation or patrol.

        Args:
            timeout: Timeout [s] for reaching the robot. Defaults to the constructor value.

        Raises:
            CubePetitNotRunning: If the navigation stack is unreachable within the timeout.
        """
        timeout = self._resolve_timeout(timeout)
        self._ensure_open()
        publisher = self._node.create_publisher(String, petit_names.navigation_cancel_topic(self._namespace), 10)
        try:
            self._wait_for_subscriber(publisher, timeout, petit_names.navigation_cancel_topic(self._namespace))
            publisher.publish(String(data='cancel'))
        finally:
            self._node.destroy_publisher(publisher)

    def where_am_i(self, *, timeout: typing.Optional[float] = None) -> RobotPose:
        """Return the robot pose on the map frame.

        The pose comes from TF (``map`` -> ``base_link``); this robot does not
        publish ``amcl_pose``.

        Args:
            timeout: Timeout [s] for the TF lookup. Defaults to the constructor value.

        Returns:
            The current pose as ``RobotPose(x, y, yaw)``.

        Raises:
            CubePetitNotRunning: If no map pose is available within the timeout.
        """
        timeout = self._resolve_timeout(timeout)
        self._ensure_open()
        with self._lock:
            if self._tf_buffer is None:
                self._tf_buffer = Buffer()
                self._tf_listener = TransformListener(self._tf_buffer, self._node)
        deadline = time.monotonic() + timeout
        while True:
            try:
                transform = self._tf_buffer.lookup_transform('map',
                                                             'base_link',
                                                             rclpy.time.Time(),
                                                             timeout=Duration(seconds=0.0))
                translation = transform.transform.translation
                rotation = transform.transform.rotation
                yaw = petit_names.yaw_from_quaternion(rotation.x, rotation.y, rotation.z, rotation.w)
                return RobotPose(x=translation.x, y=translation.y, yaw=yaw)
            except Exception:  # noqa: PERF203 - tf2 raises several lookup error types
                if time.monotonic() >= deadline:
                    raise CubePetitNotRunning('地図上の位置(TF map->base_link)が取得できません。'
                                              'ナビゲーションが起動しているか確認してね') from None
                time.sleep(_POLL_INTERVAL)

    def remember_place(self, name: str, category: str = 'place', *, timeout: typing.Optional[float] = None) -> bool:
        """Save the current position as a named place.

        Args:
            name: Place name (e.g. ``kitchen``).
            category: Place category. Defaults to ``place``.
            timeout: Timeout [s] for reaching the robot. Defaults to the constructor value.

        Returns:
            True if the place was saved.

        Raises:
            ValueError: If the name is empty.
            CubePetitNotRunning: If the save place service is unreachable within the timeout.
        """
        if not isinstance(name, str) or not name.strip():
            raise ValueError('name must be a non-empty string')
        timeout = self._resolve_timeout(timeout)
        self._ensure_open()
        client = self._node.create_client(SavePlace, petit_names.save_place_service(self._namespace))
        try:
            if not client.wait_for_service(timeout_sec=timeout):
                service_name = petit_names.save_place_service(self._namespace)
                raise CubePetitNotRunning(f'service {service_name} が見つかりません')
            request = SavePlace.Request()
            request.name = name
            request.category = category
            response = self._wait_future(client.call_async(request), timeout, '場所の保存')
            return response.success
        finally:
            self._node.destroy_client(client)

    # =================================================
    # Face
    # =================================================

    def set_face(self, expression: str = 'normal', *, timeout: typing.Optional[float] = None) -> None:
        """Change the facial expression.

        Args:
            expression: One of ``normal`` / ``happy`` / ``angry`` / ``sad`` / ``puzzled``.
            timeout: Timeout [s] for reaching the robot. Defaults to the constructor value.

        Raises:
            ValueError: If the expression is unknown.
            CubePetitNotRunning: If the facial animation node is unreachable within the timeout.
        """
        expression = petit_names.normalize_face_expression(expression)
        timeout = self._resolve_timeout(timeout)
        self._ensure_open()
        publisher = self._node.create_publisher(FaceExpression, petit_names.face_command_topic(self._namespace), 10)
        try:
            self._wait_for_subscriber(publisher, timeout, petit_names.face_command_topic(self._namespace))
            publisher.publish(FaceExpression(expression=expression))
        finally:
            self._node.destroy_publisher(publisher)

    # =================================================
    # Internals
    # =================================================

    def _spin(self) -> None:
        """Spin the private executor until shutdown (runs on the daemon thread)."""
        try:
            self._executor.spin()
        except (ExternalShutdownException, RuntimeError):
            pass

    def _ensure_open(self) -> None:
        """Raise if the instance has been closed.

        Raises:
            CubePetitClosed: If :meth:`close` has been called.
        """
        if self._closed:
            raise CubePetitClosed()

    def _resolve_timeout(self, timeout: typing.Optional[float]) -> float:
        """Return the validated timeout, falling back to the constructor default.

        Args:
            timeout: Timeout [s] or ``None``.

        Returns:
            Timeout in seconds.
        """
        self._ensure_open()
        return self._timeout if timeout is None else petit_names.validate_timeout(timeout)

    def _get_speech_client(self) -> ActionClient:
        """Return the (cached) speech action client."""
        with self._lock:
            self._ensure_open()
            if self._speech_client is None:
                self._speech_client = ActionClient(self._node, Speech, petit_names.speech_action_name(self._namespace))
            return self._speech_client

    def _wait_future(self, future: 'rclpy.task.Future', timeout: float, what: str) -> typing.Any:  # noqa: ANN401
        """Wait for an rclpy future completed by the background executor.

        Args:
            future: Future returned by an async rclpy call.
            timeout: Maximum wait time [s].
            what: Human readable description for error messages.

        Returns:
            The future result.

        Raises:
            CubePetitNotRunning: If the future does not complete within the timeout.
        """
        event = threading.Event()
        future.add_done_callback(lambda _future: event.set())
        if not event.wait(timeout):
            future.cancel()
            raise CubePetitNotRunning(f'{what}が{timeout:.1f}秒以内に完了しませんでした')
        return future.result()

    def _wait_for_subscriber(self, publisher: 'rclpy.publisher.Publisher', timeout: float, topic: str) -> None:
        """Wait until at least one subscriber is connected to the publisher.

        Args:
            publisher: Publisher to check.
            timeout: Maximum wait time [s].
            topic: Topic name for the error message.

        Raises:
            CubePetitNotRunning: If nobody subscribes within the timeout.
        """
        deadline = time.monotonic() + timeout
        while publisher.get_subscription_count() == 0:
            if time.monotonic() >= deadline:
                raise CubePetitNotRunning(f'topic {topic} の購読者が見つかりません')
            time.sleep(_POLL_INTERVAL)
