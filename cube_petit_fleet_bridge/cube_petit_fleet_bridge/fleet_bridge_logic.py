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
"""Pure (ROS- and zenoh-free) helpers for the zenoh fleet connector.

Kept free of rclpy/zenoh imports so it can be unit tested with plain pytest
in the same CI job as cube_petit_python_api/petit_names.py, which this file
mirrors: zenoh_connector.py owns the ROS/zenoh glue, this module owns key
building, JSON (de)serialization, and small geometry helpers.

Zenoh key convention (fixed by fleet_adapter_zenoh, do not change):

    robots/<robot_name>/pose                  (pub)  {"x","y","yaw"}
    robots/<robot_name>/battery                (pub)  float in [0, 1]
    robots/<robot_name>/map_name                (pub)  JSON string
    robots/<robot_name>/command_is_completed    (pub)  {"id","is_completed","success"}
    robots/<robot_name>/command                (sub)  {"method","args","id"}
"""

from __future__ import annotations

import json
import math
import pathlib
import typing

#: Methods accepted on the `command` key (fleet_adapter_zenoh -> robot).
SUPPORTED_METHODS: typing.Tuple[str, ...] = (
    'move_to_pose',
    'localize',
    'dock',
    'undock',
    'speak',
    'cancel_command',
)


class CommandError(ValueError):
    """A `command` payload was malformed or failed validation."""


# =================================================
# Zenoh key building
# =================================================


def robot_key(robot_name: str, suffix: str) -> str:
    """Build a zenoh key of the form ``robots/<robot_name>/<suffix>``.

    Args:
        robot_name: Robot namespace, e.g. ``cube_petit_orange``.
        suffix: One of ``pose`` / ``battery`` / ``map_name`` /
            ``command_is_completed`` / ``command``.

    Returns:
        The full zenoh key expression.
    """
    return f'robots/{robot_name}/{suffix}'


# =================================================
# JSON (de)serialization
# =================================================


def decode_json(payload: typing.Union[bytes, bytearray, str]) -> dict:
    """Decode a zenoh payload as a JSON object.

    Args:
        payload: Raw zenoh payload, as ``bytes``/``bytearray`` or ``str``.

    Returns:
        The decoded JSON object.

    Raises:
        CommandError: If the payload is not valid JSON or not a JSON object.
    """
    text = payload.decode('utf-8') if isinstance(payload, (bytes, bytearray)) else payload
    try:
        data = json.loads(text)
    except (ValueError, TypeError) as error:
        raise CommandError(f'Invalid JSON payload: {text!r} ({error})') from None
    if not isinstance(data, dict):
        raise CommandError(f'Expected a JSON object, got {type(data).__name__}: {text!r}')
    return data


def parse_command(payload: typing.Union[bytes, bytearray, str]) -> typing.Tuple[str, dict, str]:
    """Parse an incoming ``robots/<robot>/command`` payload.

    Args:
        payload: Raw zenoh payload.

    Returns:
        ``(method, args, command_id)``.

    Raises:
        CommandError: If the payload is malformed or missing required keys.
    """
    data = decode_json(payload)
    method = data.get('method')
    if not isinstance(method, str) or not method:
        raise CommandError(f"Command missing a non-empty string 'method': {data!r}")
    command_id = data.get('id')
    if not isinstance(command_id, str) or not command_id:
        raise CommandError(f"Command missing a non-empty string 'id': {data!r}")
    args = data.get('args', {})
    if not isinstance(args, dict):
        raise CommandError(f"Command 'args' must be a JSON object: {data!r}")
    return method, args, command_id


def encode_pose(x: float, y: float, yaw: float) -> str:
    """Encode a map-frame pose as the JSON payload for the ``pose`` key."""
    return json.dumps({'x': x, 'y': y, 'yaw': yaw})


def encode_battery(level: float) -> str:
    """Encode a battery level (0.0-1.0) as the JSON payload for the ``battery`` key."""
    return json.dumps(level)


def encode_map_name(name: str) -> str:
    """Encode a map name as the JSON payload for the ``map_name`` key."""
    return json.dumps(name)


def encode_completion(command_id: str, is_completed: bool, success: bool) -> str:
    """Encode a completion notice for the ``command_is_completed`` key."""
    return json.dumps({'id': command_id, 'is_completed': is_completed, 'success': success})


# =================================================
# Per-method argument validation
# =================================================


def validate_move_to_pose_args(args: dict) -> typing.Tuple[float, float, float, str]:
    """Validate ``move_to_pose`` args: ``{x, y, yaw, map_name}``.

    Returns:
        ``(x, y, yaw, map_name)``. ``map_name`` is ``''`` when absent.

    Raises:
        CommandError: If required numeric fields are missing/invalid.
    """
    try:
        x = float(args['x'])
        y = float(args['y'])
        yaw = float(args['yaw'])
    except (KeyError, TypeError, ValueError) as error:
        raise CommandError(f"move_to_pose requires numeric 'x'/'y'/'yaw': {args!r} ({error})") from None
    map_name = args.get('map_name', '')
    if not isinstance(map_name, str):
        raise CommandError(f"move_to_pose 'map_name' must be a string: {args!r}")
    return x, y, yaw, map_name


def validate_localize_args(args: dict) -> typing.Tuple[float, float, float, str]:
    """Validate ``localize`` args: ``{pose: {x, y, theta}, map_name}``.

    Returns:
        ``(x, y, theta, map_name)``. ``map_name`` is ``''`` when absent.

    Raises:
        CommandError: If ``pose`` is missing or its fields are invalid.
    """
    pose = args.get('pose')
    if not isinstance(pose, dict):
        raise CommandError(f"localize requires an object 'pose': {args!r}")
    try:
        x = float(pose['x'])
        y = float(pose['y'])
        theta = float(pose['theta'])
    except (KeyError, TypeError, ValueError) as error:
        raise CommandError(f"localize 'pose' requires numeric 'x'/'y'/'theta': {pose!r} ({error})") from None
    map_name = args.get('map_name', '')
    if not isinstance(map_name, str):
        raise CommandError(f"localize 'map_name' must be a string: {args!r}")
    return x, y, theta, map_name


def validate_speak_args(args: dict) -> str:
    """Validate ``speak`` args: ``{text}``.

    Returns:
        The text to speak.

    Raises:
        CommandError: If ``text`` is missing or empty.
    """
    text = args.get('text')
    if not isinstance(text, str) or not text.strip():
        raise CommandError(f"speak requires a non-empty string 'text': {args!r}")
    return text


# =================================================
# CubePetit-side wire formats
# =================================================


def build_move_goal_text(x: float, y: float, yaw: float) -> str:
    """Build the ``std_msgs/String`` payload navigation_api_node expects on ``navigation/goal``."""
    return f'pose:{x},{y},{yaw}'


def derive_map_name_from_yaml_path(path: str) -> str:
    """Derive a short map name from a nav2 map yaml path.

    map_router.pyが保存するマップは``<MAP_BASE_DIR>/<map_name>/map.yaml``という構造
    (ディレクトリ名がmap_name、ファイル名は固定で"map.yaml")なので、ファイルの
    stemではなく親ディレクトリ名を使う必要がある。ファイルステムを直接map_name
    として使うレイアウト(``.../map/test/test.yaml``)にも親ディレクトリ名で対応
    できる(どちらもディレクトリ名==意図したmap_name)。

    Args:
        path: Full path such as ``/home/cube-petit/map/arisan_room/map.yaml``.

    Returns:
        The parent directory name (e.g. ``arisan_room``), or ``'unknown'`` if
        that and the file stem are both empty.
    """
    parsed = pathlib.Path(path)
    return parsed.parent.name or parsed.stem or 'unknown'


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """Extract the yaw angle from a quaternion (matches petit_names.yaw_from_quaternion).

    Args:
        x: Quaternion x.
        y: Quaternion y.
        z: Quaternion z.
        w: Quaternion w.

    Returns:
        Yaw angle in radians within ``[-pi, pi]``.
    """
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def quaternion_from_yaw(yaw: float) -> typing.Tuple[float, float, float, float]:
    """Build a 2D (Z-axis-only) quaternion from a yaw angle.

    Args:
        yaw: Yaw angle in radians.

    Returns:
        ``(x, y, z, w)``, with ``x == y == 0`` since this is a planar rotation.
    """
    half = yaw / 2.0
    return 0.0, 0.0, math.sin(half), math.cos(half)
