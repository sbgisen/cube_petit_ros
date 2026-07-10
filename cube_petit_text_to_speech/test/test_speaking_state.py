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
"""Unit tests for cube_petit_text_to_speech.utils.speaking_state (no ROS environment required)."""

from cube_petit_text_to_speech.utils.speaking_state import SpeakingState


class TestSpeakingState:
    """Tests for SpeakingState."""

    def test_initially_not_speaking(self) -> None:
        published: list[bool] = []
        state = SpeakingState(published.append)
        assert state.is_speaking is False
        assert published == []

    def test_start_publishes_true(self) -> None:
        published: list[bool] = []
        state = SpeakingState(published.append)
        state.start()
        assert state.is_speaking is True
        assert published == [True]

    def test_stop_after_start_publishes_false(self) -> None:
        published: list[bool] = []
        state = SpeakingState(published.append)
        state.start()
        state.stop()
        assert state.is_speaking is False
        assert published == [True, False]

    def test_stop_without_start_is_noop(self) -> None:
        """Guards the "validation failed before playback began" path: no spurious publish."""
        published: list[bool] = []
        state = SpeakingState(published.append)
        state.stop()
        assert state.is_speaking is False
        assert published == []

    def test_stop_is_idempotent(self) -> None:
        """Simulates the error/cancel path where stop() runs unconditionally in `finally`."""
        published: list[bool] = []
        state = SpeakingState(published.append)
        state.start()
        state.stop()
        state.stop()
        assert published == [True, False]

    def test_multiple_speech_cycles(self) -> None:
        published: list[bool] = []
        state = SpeakingState(published.append)
        state.start()
        state.stop()
        state.start()
        state.stop()
        assert published == [True, False, True, False]

    def test_stop_runs_even_when_exception_raised_between_start_and_stop(self) -> None:
        """Mirrors the server's try/finally: stop() must still fire on an exception."""
        published: list[bool] = []
        state = SpeakingState(published.append)
        try:
            state.start()
            raise RuntimeError('simulated playback error')
        except RuntimeError:
            pass
        finally:
            state.stop()
        assert published == [True, False]
        assert state.is_speaking is False
