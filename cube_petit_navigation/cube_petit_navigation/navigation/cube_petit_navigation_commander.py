#!/usr/bin/env python

# Copyright (c) 2025 SoftBank Corp.
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

from __future__ import annotations

import threading
from typing import Optional

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.task import Future


class CubePetitNavigationCommander:
    """Minimal Nav2 wrapper for Cube petit navigation control."""

    def __init__(self, node: Node) -> None:
        self._node: Node = node
        self._client: ActionClient = ActionClient(
            node,
            NavigateToPose,
            'navigate_to_pose',
        )

        self._goal_handle = None
        self._result: Optional[bool] = None
        self._lock = threading.Lock()

        self._node.get_logger().info('Waiting for NavigateToPose action server...')
        self._client.wait_for_server()

    def go_to_pose(self, pose: PoseStamped) -> None:
        """Send a navigation goal to Nav2."""
        goal = NavigateToPose.Goal()
        goal.pose = pose

        future = self._client.send_goal_async(
            goal,
            feedback_callback=self._on_feedback,
        )
        future.add_done_callback(self._on_goal_response)

    def cancel(self) -> None:
        """Cancel the current navigation goal, if any."""
        with self._lock:
            if self._goal_handle is not None:
                self._goal_handle.cancel_goal_async()

    def is_navigating(self) -> bool:
        """Return True if a navigation goal is currently active."""
        with self._lock:
            return self._goal_handle is not None and self._result is None

    def get_result(self) -> Optional[bool]:
        """Return the last navigation result.

        Returns:
            True if succeeded, False if failed, None if still running.
        """
        with self._lock:
            return self._result

    def _on_goal_response(self, future: Future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self._node.get_logger().warning('Navigation goal was rejected')
            return

        with self._lock:
            self._goal_handle = goal_handle
            self._result = None

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_result)

    def _on_result(self, future: Future) -> None:
        result = future.result()
        status = result.status

        with self._lock:
            self._result = status == GoalStatus.STATUS_SUCCEEDED
            self._goal_handle = None

    def _on_feedback(self, _feedback_msg: NavigateToPose.FeedbackMessage) -> None:
        """Handle navigation feedback.

        Feedback is intentionally ignored in the current Cube petit design.
        """
        return
