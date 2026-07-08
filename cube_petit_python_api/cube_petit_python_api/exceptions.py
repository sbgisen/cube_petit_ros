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
"""Exceptions for the CubePetit facade API (no ROS dependency)."""

from __future__ import annotations


class CubePetitError(RuntimeError):
    """Base error raised by the CubePetit facade API."""


class CubePetitNotRunning(CubePetitError):  # noqa: N818 - friendly name is part of the public API
    """Raised when the robot stack does not respond within the timeout."""

    def __init__(self, detail: str = '') -> None:
        """Build a friendly error message.

        Args:
            detail: Extra context appended to the message (e.g. which interface timed out).
        """
        message = ('ロボットが起動していないみたい。'
                   'cube_petit の起動状態・ネットワーク・namespace 設定を確認してね。')
        if detail:
            message = f'{message} ({detail})'
        super().__init__(message)


class CubePetitClosed(CubePetitError):  # noqa: N818 - friendly name is part of the public API
    """Raised when a method is called on a closed CubePetit instance."""

    def __init__(self) -> None:
        """Build the error message."""
        super().__init__('この CubePetit インスタンスはすでに close() されています。新しく作り直してね。')
