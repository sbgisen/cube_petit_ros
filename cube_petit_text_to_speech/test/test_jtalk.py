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
"""Unit tests for cube_petit_text_to_speech.utils.jtalk (no ROS environment required)."""

import pathlib

from cube_petit_text_to_speech.utils import jtalk
from cube_petit_text_to_speech.utils.jtalk import adjust_text
from cube_petit_text_to_speech.utils.jtalk import check_goal
from cube_petit_text_to_speech.utils.jtalk import generate_jtalk_command
import pytest

SUFFIX = '〜っ、。'


class TestAdjustText:
    """Tests for adjust_text()."""

    def test_empty_string_gets_only_suffix(self) -> None:
        assert adjust_text('') == SUFFIX

    def test_plain_japanese_text_is_kept(self) -> None:
        assert adjust_text('こんにちは') == 'こんにちは' + SUFFIX

    def test_half_width_space_becomes_comma(self) -> None:
        assert adjust_text('a b') == 'a、b' + SUFFIX

    def test_full_width_space_becomes_comma(self) -> None:
        assert adjust_text('あ　い') == 'あ、い' + SUFFIX

    def test_newline_becomes_period(self) -> None:
        assert adjust_text('あ\nい') == 'あ。い' + SUFFIX

    def test_repeat_mark_becomes_long_vowel(self) -> None:
        assert adjust_text('ゝ') == 'ー、' + SUFFIX

    def test_corner_brackets_become_commas(self) -> None:
        assert adjust_text('「こんにちは」') == '、こんにちは、' + SUFFIX

    def test_full_width_exclamation_mark(self) -> None:
        assert adjust_text('！') == 'っ。' + SUFFIX

    def test_half_width_exclamation_mark(self) -> None:
        assert adjust_text('Hello!') == 'Helloっ。' + SUFFIX

    def test_periods_and_commas_are_normalized(self) -> None:
        assert adjust_text('．，') == '。、' + SUFFIX

    def test_half_width_period_between_digits(self) -> None:
        assert adjust_text('1.5') == '1。5' + SUFFIX

    def test_ascii_letters_and_digits_are_kept(self) -> None:
        assert adjust_text('数字123') == '数字123' + SUFFIX

    def test_half_width_symbol_becomes_comma(self) -> None:
        assert adjust_text('#') == '、' + SUFFIX

    def test_full_width_digits_become_commas(self) -> None:
        assert adjust_text('１２３') == '、、、' + SUFFIX

    def test_duplicated_periods_are_collapsed(self) -> None:
        assert adjust_text('。。') == '。' + SUFFIX

    def test_period_followed_by_space_is_collapsed(self) -> None:
        assert adjust_text('おはよう。 ') == 'おはよう。' + SUFFIX

    def test_horizontal_bar_is_removed(self) -> None:
        assert adjust_text('―') == SUFFIX

    def test_mixed_sentence(self) -> None:
        assert adjust_text('Hello, world!') == 'Hello、、worldっ。' + SUFFIX

    def test_non_string_input_is_rejected(self) -> None:
        with pytest.raises(Exception):
            adjust_text(123)


class TestCheckGoal:
    """Tests for check_goal()."""

    def test_valid_goal(self) -> None:
        assert check_goal('こんにちは', 'happy', 3, 100, 100, 50) is True

    @pytest.mark.parametrize('emotion', ['happy', 'normal', 'angry', 'bashful', 'sad'])
    def test_all_valid_emotions(self, emotion: str) -> None:
        assert check_goal('hi', emotion, 1, 50, 50, 1) is True

    def test_empty_text_is_invalid(self) -> None:
        assert check_goal('', 'happy', 3, 100, 100, 50) is False

    @pytest.mark.parametrize('emotion', ['', 'default', 'happiness', 'HAPPY'])
    def test_unknown_emotion_is_invalid(self, emotion: str) -> None:
        assert check_goal('hi', emotion, 3, 100, 100, 50) is False

    @pytest.mark.parametrize('emotion_level, expected', [(0, False), (1, True), (5, True), (6, False)])
    def test_emotion_level_bounds(self, emotion_level: int, expected: bool) -> None:
        assert check_goal('hi', 'happy', emotion_level, 100, 100, 50) is expected

    @pytest.mark.parametrize('pitch, expected', [(49, False), (50, True), (199, True), (200, False)])
    def test_pitch_bounds(self, pitch: int, expected: bool) -> None:
        assert check_goal('hi', 'happy', 3, pitch, 100, 50) is expected

    @pytest.mark.parametrize('speed, expected', [(49, False), (50, True), (299, True), (300, False)])
    def test_speed_bounds(self, speed: int, expected: bool) -> None:
        assert check_goal('hi', 'happy', 3, 100, speed, 50) is expected

    @pytest.mark.parametrize('volume, expected', [(0, False), (1, True), (100, True), (101, False)])
    def test_volume_bounds(self, volume: int, expected: bool) -> None:
        assert check_goal('hi', 'happy', 3, 100, 100, volume) is expected


class TestGenerateJtalkCommand:
    """Tests for generate_jtalk_command()."""

    def test_default_output_file(self) -> None:
        assert generate_jtalk_command() == ['pw-play', '/tmp/jtalk_output.wav']

    def test_custom_file_path(self) -> None:
        assert generate_jtalk_command(pathlib.Path('/tmp/x.wav')) == ['pw-play', '/tmp/x.wav']

    def test_options_are_appended(self) -> None:
        command = generate_jtalk_command(pathlib.Path('/tmp/x.wav'), ['--volume', '50'])
        assert command == ['pw-play', '/tmp/x.wav', '--volume', '50']

    def test_default_options_are_not_shared_between_calls(self) -> None:
        first = generate_jtalk_command()
        first.append('--tainted')
        assert generate_jtalk_command() == ['pw-play', '/tmp/jtalk_output.wav']


class TestLazyLibRoute:
    """Tests for the lazy LIB_ROUTE module attribute."""

    def test_unknown_attribute_raises(self) -> None:
        with pytest.raises(AttributeError):
            jtalk.no_such_attribute

    def test_lib_route_is_exported(self) -> None:
        assert 'LIB_ROUTE' in jtalk.__all__
