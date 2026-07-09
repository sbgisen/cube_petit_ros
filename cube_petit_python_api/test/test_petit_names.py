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
"""Unit tests for the pure-logic helpers of the CubePetit facade (no ROS required)."""

import math

import pytest

from cube_petit_python_api import petit_names
from cube_petit_python_api.exceptions import CubePetitError
from cube_petit_python_api.exceptions import CubePetitNotRunning


class TestResolveRobotNamespace:
    """Tests for resolve_robot_namespace()."""

    def test_default(self, monkeypatch: pytest.MonkeyPatch) -> None:
        monkeypatch.delenv(petit_names.ROBOT_NS_ENV, raising=False)
        assert petit_names.resolve_robot_namespace() == 'cube_petit_orange'

    def test_env_variable(self, monkeypatch: pytest.MonkeyPatch) -> None:
        monkeypatch.setenv(petit_names.ROBOT_NS_ENV, 'cube_petit_pink')
        assert petit_names.resolve_robot_namespace() == 'cube_petit_pink'

    def test_argument_wins_over_env(self, monkeypatch: pytest.MonkeyPatch) -> None:
        monkeypatch.setenv(petit_names.ROBOT_NS_ENV, 'cube_petit_pink')
        assert petit_names.resolve_robot_namespace('cube_petit_orange') == 'cube_petit_orange'

    def test_strips_slashes(self) -> None:
        assert petit_names.resolve_robot_namespace('/cube_petit_orange/') == 'cube_petit_orange'

    @pytest.mark.parametrize('bad', ['', '/', 'has space', '0start', 'a//b', 'にほんご'])
    def test_invalid(self, bad: str) -> None:
        with pytest.raises(ValueError):
            petit_names.resolve_robot_namespace(bad)

    def test_non_string(self) -> None:
        with pytest.raises(ValueError):
            petit_names.resolve_robot_namespace(123)  # type: ignore[arg-type]


class TestNameBuilders:
    """Tests for the fully-qualified name builders."""

    def test_speech_action_name(self) -> None:
        assert petit_names.speech_action_name('cube_petit_orange') == '/cube_petit_orange/speech_action_server'

    def test_face_command_topic(self) -> None:
        assert petit_names.face_command_topic('ns') == '/ns/facial_expression/expression_command'

    def test_navigation_topics(self) -> None:
        assert petit_names.navigation_goal_topic('ns') == '/ns/navigation/goal'
        assert petit_names.navigation_cancel_topic('ns') == '/ns/navigation/cancel'
        assert petit_names.navigation_status_topic('ns') == '/ns/navigation/status'
        assert petit_names.save_place_service('ns') == '/ns/navigation/save_place'


class TestNormalizeSayEmotion:
    """Tests for normalize_say_emotion()."""

    @pytest.mark.parametrize(('alias', 'expected'), [
        ('normal', 'default'),
        ('happy', 'happiness'),
        ('angry', 'anger'),
        ('sad', 'sadness'),
        ('shout', 'shout'),
        ('HAPPY', 'happiness'),
        ('default', 'default'),
    ])
    def test_alias(self, alias: str, expected: str) -> None:
        assert petit_names.normalize_say_emotion(alias) == expected

    def test_unknown(self) -> None:
        with pytest.raises(ValueError):
            petit_names.normalize_say_emotion('sleepy')


class TestNormalizeFaceExpression:
    """Tests for normalize_face_expression()."""

    @pytest.mark.parametrize('expression', ['normal', 'happy', 'angry', 'sad', 'puzzled'])
    def test_valid(self, expression: str) -> None:
        assert petit_names.normalize_face_expression(expression) == expression

    def test_unknown(self) -> None:
        with pytest.raises(ValueError):
            petit_names.normalize_face_expression('crying')


class TestBuildMoveGoal:
    """Tests for build_move_goal()."""

    @pytest.mark.parametrize('keyword', ['favorite', 'patrol'])
    def test_keyword(self, keyword: str) -> None:
        assert petit_names.build_move_goal(keyword) == keyword

    def test_pose_tuple(self) -> None:
        assert petit_names.build_move_goal((1, 2.5, 0)) == 'pose:1.0,2.5,0.0'

    def test_pose_string(self) -> None:
        assert petit_names.build_move_goal('pose:1.0,2.0,0.5') == 'pose:1.0,2.0,0.5'

    @pytest.mark.parametrize('bad', ['kitchen', 'pose:1,2', 'pose:a,b,c', (1, 2), (1, 2, 'x'), None, 42])
    def test_invalid(self, bad: object) -> None:
        with pytest.raises(ValueError):
            petit_names.build_move_goal(bad)  # type: ignore[arg-type]


class TestValidateTimeout:
    """Tests for validate_timeout()."""

    def test_valid(self) -> None:
        assert petit_names.validate_timeout(3) == 3.0
        assert petit_names.validate_timeout(0.5) == 0.5

    @pytest.mark.parametrize('bad', [0, -1, float('inf'), float('nan'), 'fast', None, True])
    def test_invalid(self, bad: object) -> None:
        with pytest.raises(ValueError):
            petit_names.validate_timeout(bad)  # type: ignore[arg-type]


class TestYawFromQuaternion:
    """Tests for yaw_from_quaternion()."""

    def test_identity(self) -> None:
        assert petit_names.yaw_from_quaternion(0.0, 0.0, 0.0, 1.0) == pytest.approx(0.0)

    def test_quarter_turn(self) -> None:
        half = math.sin(math.pi / 4.0)
        assert petit_names.yaw_from_quaternion(0.0, 0.0, half, half) == pytest.approx(math.pi / 2.0)


class TestExceptions:
    """Tests for the friendly exceptions."""

    def test_not_running_message(self) -> None:
        error = CubePetitNotRunning('speech が見つかりません')
        assert 'ロボットが起動していないみたい' in str(error)
        assert 'speech が見つかりません' in str(error)
        assert isinstance(error, CubePetitError)

    def test_lazy_package_import_is_ros_free(self) -> None:
        import cube_petit_python_api
        assert 'CubePetit' in cube_petit_python_api.__all__
