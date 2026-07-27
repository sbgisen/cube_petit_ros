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
"""Plain pytest tests for shared_controller_logic (no rclpy / no zenoh required)."""

import json

import pytest

from cube_petit_shared_controller import shared_controller_logic as logic


class TestSelectedRobotCodec:

    def test_round_trip(self) -> None:
        payload = logic.encode_selected_robots(['cube_petit_pink', 'cube_petit_orange'])
        assert json.loads(payload) == {'robot_names': ['cube_petit_pink', 'cube_petit_orange']}
        assert logic.decode_selected_robots(payload) == ['cube_petit_pink', 'cube_petit_orange']

    def test_round_trip_empty(self) -> None:
        payload = logic.encode_selected_robots([])
        assert logic.decode_selected_robots(payload) == []

    def test_decode_accepts_bytes(self) -> None:
        assert logic.decode_selected_robots(b'{"robot_names": ["cube_petit_orange"]}') == ['cube_petit_orange']

    def test_decode_rejects_invalid_json(self) -> None:
        with pytest.raises(logic.ControllerMessageError):
            logic.decode_selected_robots('not json')

    def test_decode_rejects_missing_field(self) -> None:
        with pytest.raises(logic.ControllerMessageError):
            logic.decode_selected_robots('{}')

    def test_decode_rejects_non_list(self) -> None:
        with pytest.raises(logic.ControllerMessageError):
            logic.decode_selected_robots('{"robot_names": "cube_petit_orange"}')

    def test_decode_rejects_empty_name_in_list(self) -> None:
        with pytest.raises(logic.ControllerMessageError):
            logic.decode_selected_robots('{"robot_names": [""]}')

    def test_decode_rejects_non_string_name_in_list(self) -> None:
        with pytest.raises(logic.ControllerMessageError):
            logic.decode_selected_robots('{"robot_names": [1]}')


class TestCmdVelCodec:

    def test_round_trip(self) -> None:
        payload = logic.encode_cmd_vel(0.3, -1.5)
        assert json.loads(payload) == {'linear_x': 0.3, 'angular_z': -1.5}
        assert logic.decode_cmd_vel(payload) == (0.3, -1.5)

    def test_decode_accepts_bytes(self) -> None:
        assert logic.decode_cmd_vel(b'{"linear_x": 1, "angular_z": 2}') == (1.0, 2.0)

    def test_decode_rejects_invalid_json(self) -> None:
        with pytest.raises(logic.ControllerMessageError):
            logic.decode_cmd_vel('nope')

    def test_decode_rejects_missing_field(self) -> None:
        with pytest.raises(logic.ControllerMessageError):
            logic.decode_cmd_vel('{"linear_x": 1.0}')

    def test_decode_rejects_non_numeric_field(self) -> None:
        with pytest.raises(logic.ControllerMessageError):
            logic.decode_cmd_vel('{"linear_x": "fast", "angular_z": 0.0}')


class TestToggleRobotSelection:

    def test_adds_when_absent(self) -> None:
        assert logic.toggle_robot_selection(frozenset(), 'cube_petit_pink') == frozenset({'cube_petit_pink'})

    def test_removes_when_present(self) -> None:
        selected = frozenset({'cube_petit_pink', 'cube_petit_orange'})
        assert logic.toggle_robot_selection(selected, 'cube_petit_pink') == frozenset({'cube_petit_orange'})

    def test_does_not_mutate_other_members(self) -> None:
        selected = frozenset({'cube_petit_orange'})
        result = logic.toggle_robot_selection(selected, 'cube_petit_pink')
        assert result == frozenset({'cube_petit_orange', 'cube_petit_pink'})


class TestExclusiveRobotSelection:

    def test_selects_only_that_robot(self) -> None:
        assert logic.exclusive_robot_selection('cube_petit_pink') == frozenset({'cube_petit_pink'})


class TestButtonRisingEdge:

    def test_rising_edge_detected(self) -> None:
        assert logic.button_rising_edge([0, 0, 0], [0, 0, 1], 2) is True

    def test_already_held_is_not_an_edge(self) -> None:
        assert logic.button_rising_edge([0, 0, 1], [0, 0, 1], 2) is False

    def test_release_is_not_an_edge(self) -> None:
        assert logic.button_rising_edge([0, 0, 1], [0, 0, 0], 2) is False

    def test_first_message_with_no_previous_state(self) -> None:
        # previous_buttons=() covers the very first Joy message (no prior state yet).
        assert logic.button_rising_edge((), [0, 0, 1], 2) is True
        assert logic.button_rising_edge((), [0, 0, 0], 2) is False

    def test_out_of_range_index_is_never_an_edge(self) -> None:
        assert logic.button_rising_edge([0, 0], [0, 0], 5) is False


class TestResolveRobotName:

    def test_param_wins(self) -> None:
        assert logic.resolve_robot_name('cube_petit_pink', 'cube_petit_orange', '/cube_petit_blue',
                                        'cube_petit') == 'cube_petit_pink'

    def test_env_wins_over_namespace(self) -> None:
        assert logic.resolve_robot_name('', 'cube_petit_orange', '/cube_petit_blue', 'cube_petit') \
            == 'cube_petit_orange'

    def test_namespace_wins_over_fallback(self) -> None:
        assert logic.resolve_robot_name('', None, '/cube_petit_blue', 'cube_petit') == 'cube_petit_blue'

    def test_fallback_when_nothing_set(self) -> None:
        assert logic.resolve_robot_name('', None, '/', 'cube_petit') == 'cube_petit'
