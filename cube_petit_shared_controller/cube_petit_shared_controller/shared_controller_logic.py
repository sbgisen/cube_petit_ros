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
"""Pure (ROS- and zenoh-free) helpers for the shared PS4 controller relay.

Kept free of rclpy/zenoh imports so it can be unit tested with plain pytest,
mirroring cube_petit_fleet_bridge/fleet_bridge_logic.py: the *_node.py files
own the ROS/zenoh glue, this module owns key names, JSON (de)serialization,
button-edge detection and small selection-state helpers.

Zenoh key convention (this package only; unrelated to fleet_adapter_zenoh's
``robots/<robot_name>/...`` keys used by cube_petit_fleet_bridge):

    controller/selected_robot   (pub by hub, sub by every receiver)
        {"robot_name": "cube_petit_pink"}
    controller/cmd_vel          (pub by hub, sub by every receiver)
        {"linear_x": float, "angular_z": float}
"""

from __future__ import annotations

import json
import typing

#: zenoh key the hub publishes the currently selected robot name to.
SELECTED_ROBOT_KEY = 'controller/selected_robot'
#: zenoh key the hub publishes the joystick-derived velocity command to.
CMD_VEL_KEY = 'controller/cmd_vel'


class ControllerMessageError(ValueError):
    """A ``controller/selected_robot`` or ``controller/cmd_vel`` payload was malformed."""


# =================================================
# JSON (de)serialization
# =================================================


def _decode_json_object(payload: typing.Union[bytes, bytearray, str]) -> dict:
    """Decode a zenoh payload as a JSON object.

    Args:
        payload: Raw zenoh payload, as ``bytes``/``bytearray`` or ``str``.

    Returns:
        The decoded JSON object.

    Raises:
        ControllerMessageError: If the payload is not valid JSON or not a JSON object.
    """
    text = payload.decode('utf-8') if isinstance(payload, (bytes, bytearray)) else payload
    try:
        data = json.loads(text)
    except (ValueError, TypeError) as error:
        raise ControllerMessageError(f'Invalid JSON payload: {text!r} ({error})') from None
    if not isinstance(data, dict):
        raise ControllerMessageError(f'Expected a JSON object, got {type(data).__name__}: {text!r}')
    return data


def encode_selected_robot(robot_name: str) -> str:
    """Encode the currently selected robot name as the ``controller/selected_robot`` payload."""
    return json.dumps({'robot_name': robot_name})


def decode_selected_robot(payload: typing.Union[bytes, bytearray, str]) -> str:
    """Decode a ``controller/selected_robot`` payload.

    Args:
        payload: Raw zenoh payload.

    Returns:
        The selected robot name.

    Raises:
        ControllerMessageError: If the payload is malformed or ``robot_name`` is missing/empty.
    """
    data = _decode_json_object(payload)
    robot_name = data.get('robot_name')
    if not isinstance(robot_name, str) or not robot_name:
        raise ControllerMessageError(f"Expected a non-empty string 'robot_name': {data!r}")
    return robot_name


def encode_cmd_vel(linear_x: float, angular_z: float) -> str:
    """Encode a velocity command as the ``controller/cmd_vel`` payload."""
    return json.dumps({'linear_x': linear_x, 'angular_z': angular_z})


def decode_cmd_vel(payload: typing.Union[bytes, bytearray, str]) -> typing.Tuple[float, float]:
    """Decode a ``controller/cmd_vel`` payload.

    Args:
        payload: Raw zenoh payload.

    Returns:
        ``(linear_x, angular_z)``.

    Raises:
        ControllerMessageError: If the payload is malformed or the fields are not numeric.
    """
    data = _decode_json_object(payload)
    try:
        linear_x = float(data['linear_x'])
        angular_z = float(data['angular_z'])
    except (KeyError, TypeError, ValueError) as error:
        raise ControllerMessageError(f"Expected numeric 'linear_x'/'angular_z': {data!r} ({error})") from None
    return linear_x, angular_z


# =================================================
# Robot selection (hub side)
# =================================================


def next_robot_index(current_index: int, num_robots: int) -> int:
    """Compute the index to toggle to next, wrapping around ``robot_names``.

    Args:
        current_index: Index of the currently selected robot.
        num_robots: Number of candidate robots (``len(robot_names)``).

    Returns:
        The next index, ``(current_index + 1) % num_robots``.

    Raises:
        ValueError: If ``num_robots`` is not positive.
    """
    if num_robots <= 0:
        raise ValueError('num_robots must be positive (robot_names must not be empty)')
    return (current_index + 1) % num_robots


def button_rising_edge(previous_buttons: typing.Sequence[int], current_buttons: typing.Sequence[int],
                       button_index: int) -> bool:
    """Detect the rising edge (0 -> 1) of one ``sensor_msgs/msg/Joy`` button.

    Args:
        previous_buttons: ``buttons`` array from the previous ``Joy`` message (``()`` if none yet).
        current_buttons: ``buttons`` array from the current ``Joy`` message.
        button_index: Index to watch, e.g. the ``switch_button`` parameter.

    Returns:
        ``True`` only if ``button_index`` is a valid index into ``current_buttons``, its value is
        truthy (pressed), and it was not already pressed in ``previous_buttons`` (or
        ``previous_buttons`` doesn't cover that index yet, e.g. the very first ``Joy`` message).
    """
    if button_index < 0 or button_index >= len(current_buttons):
        return False
    is_pressed = bool(current_buttons[button_index])
    was_pressed = button_index < len(previous_buttons) and bool(previous_buttons[button_index])
    return is_pressed and not was_pressed


# =================================================
# Robot name resolution (receiver side)
# =================================================


def resolve_robot_name(param_value: str, env_value: typing.Optional[str], node_namespace: str, fallback: str) -> str:
    """Resolve "which robot am I" for the receiver node.

    Precedence: explicit ``robot_name`` launch parameter > ``ROBOT_NAMESPACE`` environment
    variable > the node's own ROS namespace > ``fallback``. Mirrors
    ``cube_petit_fleet_bridge.zenoh_connector.ZenohConnector``'s ``robot_name`` resolution,
    extended with the ``ROBOT_NAMESPACE`` env var this package's receiver node also accepts.

    Args:
        param_value: Value of the ``robot_name`` ROS parameter (``''`` means unset).
        env_value: Value of the ``ROBOT_NAMESPACE`` environment variable, or ``None``.
        node_namespace: The node's own ROS namespace (``get_namespace()``, e.g. ``'/cube_petit_pink'``).
        fallback: Value to use if none of the above are set.

    Returns:
        The resolved robot name, never empty.
    """
    if param_value:
        return param_value
    if env_value:
        return env_value
    stripped_namespace = node_namespace.strip('/')
    if stripped_namespace:
        return stripped_namespace
    return fallback
