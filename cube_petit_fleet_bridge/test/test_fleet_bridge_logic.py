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
"""Plain pytest tests for fleet_bridge_logic (no rclpy / no zenoh required)."""

import json
import math

import pytest

from cube_petit_fleet_bridge import fleet_bridge_logic as logic


class TestRobotKey:

    def test_builds_expected_key(self) -> None:
        assert logic.robot_key('cube_petit_orange', 'pose') == 'robots/cube_petit_orange/pose'
        assert logic.robot_key('cube_petit_pink', 'command') == 'robots/cube_petit_pink/command'


class TestDecodeJson:

    def test_accepts_bytes_and_str(self) -> None:
        assert logic.decode_json(b'{"a": 1}') == {'a': 1}
        assert logic.decode_json('{"a": 1}') == {'a': 1}

    def test_rejects_invalid_json(self) -> None:
        with pytest.raises(logic.CommandError):
            logic.decode_json('not json')

    def test_rejects_non_object(self) -> None:
        with pytest.raises(logic.CommandError):
            logic.decode_json('[1, 2, 3]')


class TestParseCommand:

    def test_valid_command(self) -> None:
        payload = json.dumps({
            'method': 'move_to_pose',
            'args': {
                'x': 1.0,
                'y': 2.0,
                'yaw': 0.0
            },
            'id': 'abc123',
        })
        method, args, command_id = logic.parse_command(payload)
        assert method == 'move_to_pose'
        assert args == {'x': 1.0, 'y': 2.0, 'yaw': 0.0}
        assert command_id == 'abc123'

    def test_defaults_missing_args_to_empty_dict(self) -> None:
        payload = json.dumps({'method': 'cancel_command', 'id': 'xyz'})
        method, args, command_id = logic.parse_command(payload)
        assert args == {}
        assert method == 'cancel_command'
        assert command_id == 'xyz'

    @pytest.mark.parametrize('payload', [
        json.dumps({
            'args': {},
            'id': 'x'
        }),
        json.dumps({
            'method': '',
            'id': 'x'
        }),
        json.dumps({
            'method': 'speak',
            'args': {}
        }),
        json.dumps({
            'method': 'speak',
            'id': ''
        }),
        json.dumps({
            'method': 'speak',
            'args': [],
            'id': 'x'
        }),
        json.dumps({
            'method': 1,
            'id': 'x'
        }),
    ])
    def test_rejects_malformed_command(self, payload: str) -> None:
        with pytest.raises(logic.CommandError):
            logic.parse_command(payload)


class TestEncoders:

    def test_encode_pose(self) -> None:
        assert json.loads(logic.encode_pose(1.5, -2.5, 0.25)) == {'x': 1.5, 'y': -2.5, 'yaw': 0.25}

    def test_encode_battery(self) -> None:
        assert json.loads(logic.encode_battery(1.0)) == 1.0

    def test_encode_map_name(self) -> None:
        assert json.loads(logic.encode_map_name('test')) == 'test'

    def test_encode_completion(self) -> None:
        decoded = json.loads(logic.encode_completion('id-1', True, False))
        assert decoded == {'id': 'id-1', 'is_completed': True, 'success': False}


class TestValidateMoveToPoseArgs:

    def test_valid(self) -> None:
        x, y, yaw, map_name = logic.validate_move_to_pose_args({'x': 1, 'y': 2, 'yaw': 3, 'map_name': 'test'})
        assert (x, y, yaw, map_name) == (1.0, 2.0, 3.0, 'test')

    def test_map_name_defaults_to_empty(self) -> None:
        _x, _y, _yaw, map_name = logic.validate_move_to_pose_args({'x': 1, 'y': 2, 'yaw': 3})
        assert map_name == ''

    @pytest.mark.parametrize('args', [
        {
            'y': 2,
            'yaw': 3
        },
        {
            'x': 'nan-ish',
            'y': 2,
            'yaw': 3
        },
        {
            'x': 1,
            'y': 2,
            'yaw': 3,
            'map_name': 42
        },
    ])
    def test_rejects_invalid(self, args: dict) -> None:
        with pytest.raises(logic.CommandError):
            logic.validate_move_to_pose_args(args)


class TestValidateLocalizeArgs:

    def test_valid(self) -> None:
        x, y, theta, map_name = logic.validate_localize_args({
            'pose': {
                'x': 1,
                'y': 2,
                'theta': 0.5
            },
            'map_name': 'test',
        })
        assert (x, y, theta, map_name) == (1.0, 2.0, 0.5, 'test')

    def test_rejects_missing_pose(self) -> None:
        with pytest.raises(logic.CommandError):
            logic.validate_localize_args({'map_name': 'test'})

    def test_rejects_incomplete_pose(self) -> None:
        with pytest.raises(logic.CommandError):
            logic.validate_localize_args({'pose': {'x': 1, 'y': 2}})


class TestValidateSpeakArgs:

    def test_valid(self) -> None:
        assert logic.validate_speak_args({'text': 'こんにちは'}) == 'こんにちは'

    @pytest.mark.parametrize('args', [{}, {'text': ''}, {'text': '   '}, {'text': 123}])
    def test_rejects_invalid(self, args: dict) -> None:
        with pytest.raises(logic.CommandError):
            logic.validate_speak_args(args)


class TestBuildMoveGoalText:

    def test_format(self) -> None:
        assert logic.build_move_goal_text(1.0, -2.5, 3.14) == 'pose:1.0,-2.5,3.14'


class TestDeriveMapNameFromYamlPath:

    def test_extracts_parent_directory_name(self) -> None:
        # map_router.pyの保存レイアウト: <MAP_BASE_DIR>/<map_name>/map.yaml
        # (ファイル名は常に"map.yaml"固定、ディレクトリ名がmap_name)。
        assert logic.derive_map_name_from_yaml_path('/home/cube-petit/map/arisan_room/map.yaml') == 'arisan_room'

    def test_extracts_parent_directory_name_when_stem_matches_too(self) -> None:
        assert logic.derive_map_name_from_yaml_path('/opt/share/cube_petit_navigation/map/test/test.yaml') == 'test'

    def test_empty_path_falls_back(self) -> None:
        assert logic.derive_map_name_from_yaml_path('') == 'unknown'


class TestQuaternionYawRoundTrip:

    @pytest.mark.parametrize('yaw', [0.0, 0.5, -0.5, 1.5707963267948966, math.pi, -math.pi + 0.01])
    def test_round_trip(self, yaw: float) -> None:
        qx, qy, qz, qw = logic.quaternion_from_yaw(yaw)
        assert qx == 0.0
        assert qy == 0.0
        recovered = logic.yaw_from_quaternion(qx, qy, qz, qw)
        assert recovered == pytest.approx(yaw, abs=1e-9)
