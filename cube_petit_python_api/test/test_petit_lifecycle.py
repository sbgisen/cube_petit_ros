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
"""Lifecycle tests for the CubePetit facade (requires rclpy, no robot).

Skipped automatically when rclpy or the message packages are unavailable
(e.g. the plain-Python unit-tests CI job). In the colcon test environment
these run without any robot: every call must fail fast with
CubePetitNotRunning instead of hanging.

Note: a plain ``pytest.importorskip`` at module level aborts the whole
directory collection with this pytest/plugin combination, hence the
try/except + ``skipif`` guard.
"""

import time

import pytest

try:
    import rclpy  # noqa: F401

    import cube_petit_facial_animation_msgs  # noqa: F401
    import cube_petit_navigation_msgs  # noqa: F401
    from cube_petit_python_api.exceptions import CubePetitClosed
    from cube_petit_python_api.exceptions import CubePetitNotRunning
    from cube_petit_python_api.petit import CubePetit
    import cube_petit_speech_msgs  # noqa: F401
    _SKIP_REASON = ''
except ImportError as error:  # pragma: no cover - exercised only without ROS
    _SKIP_REASON = f'ROS environment not available: {error}'

pytestmark = pytest.mark.skipif(bool(_SKIP_REASON), reason=_SKIP_REASON)

# Isolated ROS domain so a robot running on the same machine/network cannot
# interfere with the "stack down" assumption.
_TEST_DOMAIN_ID = 77
_TEST_ROBOT = 'cube_petit_test_no_robot'
_TIMEOUT = 1.0


def _make_robot() -> 'CubePetit':
    return CubePetit(robot=_TEST_ROBOT, timeout=_TIMEOUT, domain_id=_TEST_DOMAIN_ID)


class TestNoRobot:
    """Every API call must raise CubePetitNotRunning quickly when nothing is running."""

    def test_say_raises_within_timeout(self) -> None:
        start = time.monotonic()
        with _make_robot() as robot:
            with pytest.raises(CubePetitNotRunning):
                robot.say('こんにちは')
        assert time.monotonic() - start < _TIMEOUT + 10.0

    def test_move_where_face_raise(self) -> None:
        with _make_robot() as robot:
            with pytest.raises(CubePetitNotRunning):
                robot.move_to('favorite')
            with pytest.raises(CubePetitNotRunning):
                robot.where_am_i()
            with pytest.raises(CubePetitNotRunning):
                robot.set_face('happy')
            with pytest.raises(CubePetitNotRunning):
                robot.cancel_move()
            with pytest.raises(CubePetitNotRunning):
                robot.remember_place('kitchen')

    def test_invalid_arguments_fail_fast(self) -> None:
        with _make_robot() as robot:
            with pytest.raises(ValueError):
                robot.say('')
            with pytest.raises(ValueError):
                robot.say('hello', emotion='sleepy')
            with pytest.raises(ValueError):
                robot.move_to('unknown_place')
            with pytest.raises(ValueError):
                robot.set_face('crying')


class TestLifecycle:
    """Create/close cycles must be leak-free and repeatable in one process."""

    def test_init_close_twice(self) -> None:
        for _ in range(2):
            robot = _make_robot()
            assert robot.robot == _TEST_ROBOT
            with pytest.raises(CubePetitNotRunning):
                robot.where_am_i(timeout=0.5)
            robot.close()

    def test_close_is_idempotent(self) -> None:
        robot = _make_robot()
        robot.close()
        robot.close()

    def test_methods_after_close_raise(self) -> None:
        robot = _make_robot()
        robot.close()
        with pytest.raises(CubePetitClosed):
            robot.say('hello')
        with pytest.raises(CubePetitClosed):
            robot.where_am_i()

    def test_spin_thread_stops_after_close(self) -> None:
        robot = _make_robot()
        thread = robot._thread  # noqa: SLF001
        assert thread.is_alive()
        robot.close()
        thread.join(timeout=5.0)
        assert not thread.is_alive()
