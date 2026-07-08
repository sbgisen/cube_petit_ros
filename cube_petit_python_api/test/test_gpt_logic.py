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
"""Unit tests for gpt_logic (no ROS environment required)."""

import gpt_logic


class TestDefaultSetting:
    """Tests for the default system prompt."""

    def test_default_setting_mentions_json_key(self) -> None:
        assert 'speech_phrase' in gpt_logic.DEFAULT_SETTING
        assert 'json' in gpt_logic.DEFAULT_SETTING


class TestBuildTextContexts:
    """Tests for build_text_contexts()."""

    def test_structure(self) -> None:
        contexts = gpt_logic.build_text_contexts('system prompt', 'こんにちは')
        assert contexts == [
            {
                'role': 'system',
                'content': 'system prompt'
            },
            {
                'role': 'user',
                'content': 'こんにちは'
            },
        ]

    def test_default_setting_is_usable_as_system_prompt(self) -> None:
        contexts = gpt_logic.build_text_contexts(gpt_logic.DEFAULT_SETTING, 'hi')
        assert contexts[0]['role'] == 'system'
        assert contexts[0]['content'] == gpt_logic.DEFAULT_SETTING


class TestBuildImageContexts:
    """Tests for build_image_contexts()."""

    def test_structure(self) -> None:
        contexts = gpt_logic.build_image_contexts('system prompt', '何が見える?', 'QUJD')
        assert len(contexts) == 2
        assert contexts[0] == {'role': 'system', 'content': 'system prompt'}
        user = contexts[1]
        assert user['role'] == 'user'
        assert user['content'][0] == {'type': 'text', 'text': '何が見える?'}
        assert user['content'][1] == {'type': 'image_url', 'image_url': 'data:image/jpeg;base64,QUJD'}

    def test_image_is_embedded_as_data_uri(self) -> None:
        contexts = gpt_logic.build_image_contexts('s', 't', 'abc123==')
        image_url = contexts[1]['content'][1]['image_url']
        assert image_url.startswith('data:image/jpeg;base64,')
        assert image_url.endswith('abc123==')
