#!/usr/bin/env python
# -*- coding:utf-8 -*-

# Copyright (c) 2024 SoftBank Corp.
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
"""Pure (ROS-independent) helpers for building GPT chat requests."""

DEFAULT_SETTING = """
                出力はjson形式で{"speech_phrase": ""}のように、speech_phraseの中に返答をいれて返してください"""


def build_text_contexts(setting: str, input_text: str) -> list:
    """Build chat contexts for a text-only request.

    Args:
        setting: System prompt.
        input_text: User input text.

    Returns:
        Message list for the Chat Completions API.
    """
    return [{'role': 'system', 'content': setting}, {'role': 'user', 'content': input_text}]


def build_image_contexts(setting: str, input_text: str, base64_image: str) -> list:
    """Build chat contexts for a request that includes an image.

    Args:
        setting: System prompt.
        input_text: User input text.
        base64_image: Base64 encoded image.

    Returns:
        Message list for the Chat Completions API.
    """
    return [{
        'role': 'system',
        'content': setting
    }, {
        'role':
            'user',
        'content': [
            {
                'type': 'text',
                'text': input_text
            },
            {
                'type': 'image_url',
                'image_url': f'data:image/jpeg;base64,{base64_image}'
            },
        ],
    }]
