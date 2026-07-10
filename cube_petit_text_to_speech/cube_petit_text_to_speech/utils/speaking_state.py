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
"""Speaking-flag state transitions, kept independent of rclpy so they can be unit tested.

このモジュールはROSに依存しない(発話中フラグの遷移だけを扱う / pure Python state machine
independent of ROS, so it is unit-testable without a ROS environment).
"""

from collections.abc import Callable

from beartype import beartype

PublishFn = Callable[[bool], None]


class SpeakingState:
    """Track whether the robot is currently speaking and drive a boolean publish callback.

    ``start()`` must be called exactly once when playback begins, and ``stop()`` must be
    called when it ends -- including error/cancel paths -- so that downstream lip-sync
    subscribers never get stuck seeing "speaking" forever
    (発話開始でTrue、終了(異常系含む)でFalseに戻すための状態機械).
    ``stop()`` is idempotent: calling it without a matching ``start()`` (e.g. because goal
    validation failed before playback ever began) is a no-op and does not publish anything.
    """

    @beartype
    def __init__(self, publish: PublishFn) -> None:
        """Init speaking state.

        Args:
            publish: Callback invoked with the new boolean state whenever it changes.
        """
        self._publish = publish
        self._speaking = False

    @property
    def is_speaking(self) -> bool:
        """Whether playback is currently considered in progress."""
        return self._speaking

    def start(self) -> None:
        """Mark speaking as started and publish True."""
        self._speaking = True
        self._publish(True)

    def stop(self) -> None:
        """Mark speaking as finished and publish False, if it was actually started."""
        if not self._speaking:
            return
        self._speaking = False
        self._publish(False)
