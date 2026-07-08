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
"""Pure helpers for the CubePetit facade (no ROS dependency).

This module holds all logic that does not need ROS: resource name building,
argument validation and small geometry helpers. Keeping it ROS-free lets the
unit tests run without a ROS environment.
"""

from __future__ import annotations

import math
import os
import re
import typing

DEFAULT_ROBOT = 'cube_petit_orange'
ROBOT_NS_ENV = 'PETIT_ROBOT_NS'

_NS_PATTERN = re.compile(r'[A-Za-z][A-Za-z0-9_]*(?:/[A-Za-z][A-Za-z0-9_]*)*')

#: Friendly emotion aliases accepted by :func:`normalize_say_emotion`,
#: mapped to the emotion names understood by the speech action server (jtalk).
SAY_EMOTIONS: typing.Dict[str, str] = {
    'normal': 'default',
    'default': 'default',
    'happy': 'happiness',
    'happiness': 'happiness',
    'angry': 'anger',
    'anger': 'anger',
    'sad': 'sadness',
    'sadness': 'sadness',
    'shout': 'shout',
}

#: Expressions accepted by the facial animation node (FaceExpression.msg).
FACE_EXPRESSIONS: typing.Tuple[str, ...] = ('normal', 'happy', 'angry', 'sad', 'puzzled')

#: Keyword goals understood by navigation_api_node.
MOVE_KEYWORDS: typing.Tuple[str, ...] = ('favorite', 'patrol')


class RobotPose(typing.NamedTuple):
    """Robot pose on the map frame (2D)."""

    x: float
    y: float
    yaw: float


def resolve_robot_namespace(robot: typing.Optional[str] = None) -> str:
    """Resolve the robot namespace from an argument or the environment.

    Priority: explicit argument > ``PETIT_ROBOT_NS`` environment variable > default.

    Args:
        robot: Robot namespace such as ``cube_petit_orange``. Leading/trailing
            slashes are tolerated. ``None`` falls back to the environment.

    Returns:
        Namespace without leading/trailing slashes.

    Raises:
        ValueError: If the resolved namespace is empty or contains invalid characters.
    """
    if robot is None:
        robot = os.environ.get(ROBOT_NS_ENV) or DEFAULT_ROBOT
    if not isinstance(robot, str):
        raise ValueError(f'robot must be a string, got {type(robot).__name__}')
    namespace = robot.strip().strip('/')
    if not namespace or not _NS_PATTERN.fullmatch(namespace):
        raise ValueError(f'Invalid robot namespace: {robot!r}')
    return namespace


def speech_action_name(namespace: str) -> str:
    """Build the fully qualified speech action name.

    Args:
        namespace: Robot namespace (no slashes around it).

    Returns:
        Action name such as ``/cube_petit_orange/speech_action_server``.
    """
    return f'/{namespace}/speech_action_server'


def face_command_topic(namespace: str) -> str:
    """Build the facial expression command topic name.

    Args:
        namespace: Robot namespace.

    Returns:
        Topic name such as ``/cube_petit_orange/facial_expression/expression_command``.
    """
    return f'/{namespace}/facial_expression/expression_command'


def navigation_goal_topic(namespace: str) -> str:
    """Build the navigation goal topic name.

    Args:
        namespace: Robot namespace.

    Returns:
        Topic name such as ``/cube_petit_orange/navigation/goal``.
    """
    return f'/{namespace}/navigation/goal'


def navigation_cancel_topic(namespace: str) -> str:
    """Build the navigation cancel topic name.

    Args:
        namespace: Robot namespace.

    Returns:
        Topic name such as ``/cube_petit_orange/navigation/cancel``.
    """
    return f'/{namespace}/navigation/cancel'


def navigation_status_topic(namespace: str) -> str:
    """Build the navigation status topic name.

    Args:
        namespace: Robot namespace.

    Returns:
        Topic name such as ``/cube_petit_orange/navigation/status``.
    """
    return f'/{namespace}/navigation/status'


def save_place_service(namespace: str) -> str:
    """Build the save place service name.

    Args:
        namespace: Robot namespace.

    Returns:
        Service name such as ``/cube_petit_orange/navigation/save_place``.
    """
    return f'/{namespace}/navigation/save_place'


def normalize_say_emotion(emotion: str) -> str:
    """Map a friendly emotion alias to a speech server emotion name.

    Args:
        emotion: One of the keys of :data:`SAY_EMOTIONS` (e.g. ``normal``, ``happy``).

    Returns:
        Emotion name for the speech action goal (e.g. ``default``, ``happiness``).

    Raises:
        ValueError: If the emotion is unknown.
    """
    if not isinstance(emotion, str) or emotion.lower() not in SAY_EMOTIONS:
        raise ValueError(f'Unknown emotion: {emotion!r}. Available: {sorted(SAY_EMOTIONS)}')
    return SAY_EMOTIONS[emotion.lower()]


def normalize_face_expression(expression: str) -> str:
    """Validate a facial expression name.

    Args:
        expression: One of :data:`FACE_EXPRESSIONS`.

    Returns:
        Normalized (lower-cased) expression name.

    Raises:
        ValueError: If the expression is unknown.
    """
    if not isinstance(expression, str) or expression.lower() not in FACE_EXPRESSIONS:
        raise ValueError(f'Unknown expression: {expression!r}. Available: {list(FACE_EXPRESSIONS)}')
    return expression.lower()


def build_move_goal(target: typing.Union[str, typing.Sequence[float]]) -> str:
    """Build a navigation goal string for navigation_api_node.

    Args:
        target: Either a keyword (``favorite`` / ``patrol``), a ``pose:x,y,yaw``
            string, or a sequence of three numbers ``(x, y, yaw)``.

    Returns:
        Goal string such as ``favorite`` or ``pose:1.0,2.0,0.0``.

    Raises:
        ValueError: If the target cannot be interpreted.
    """
    if isinstance(target, str):
        text = target.strip()
        if text in MOVE_KEYWORDS:
            return text
        if text.startswith('pose:'):
            body = text.split(':', 1)[1]
            values = body.split(',')
            if len(values) == 3:
                try:
                    x, y, yaw = (float(value) for value in values)
                except ValueError:
                    raise ValueError(f'Invalid pose goal: {target!r}') from None
                return f'pose:{x},{y},{yaw}'
        raise ValueError(f'Unknown move target: {target!r}. Use {list(MOVE_KEYWORDS)} or (x, y, yaw).')
    if isinstance(target, (tuple, list)) and len(target) == 3:
        try:
            x, y, yaw = (float(value) for value in target)
        except (TypeError, ValueError):
            raise ValueError(f'Pose must contain three numbers, got {target!r}') from None
        return f'pose:{x},{y},{yaw}'
    raise ValueError(f'Unknown move target: {target!r}. Use {list(MOVE_KEYWORDS)} or (x, y, yaw).')


def validate_timeout(timeout: typing.Union[int, float], name: str = 'timeout') -> float:
    """Validate a timeout value.

    Args:
        timeout: Timeout in seconds. Must be a positive finite number.
        name: Argument name used in the error message.

    Returns:
        The timeout as ``float``.

    Raises:
        ValueError: If the timeout is not a positive finite number.
    """
    if isinstance(timeout, bool) or not isinstance(timeout, (int, float)):
        raise ValueError(f'{name} must be a number, got {type(timeout).__name__}')
    value = float(timeout)
    if not math.isfinite(value) or value <= 0.0:
        raise ValueError(f'{name} must be a positive finite number, got {timeout!r}')
    return value


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """Extract the yaw angle from a quaternion.

    Args:
        x: Quaternion x.
        y: Quaternion y.
        z: Quaternion z.
        w: Quaternion w.

    Returns:
        Yaw angle in radians within ``[-pi, pi]``.
    """
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
