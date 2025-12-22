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

from typing import List

from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.node import Node
from src.cube_petit_navigation_commander import CubePetitNavigationCommander
from std_msgs.msg import String
from tf_transformations import quaternion_from_euler


class NavigationApiNode(Node):
    """Navigation API node that translates semantic goals into Nav2 actions."""

    def __init__(self) -> None:
        super().__init__('navigation_api_node')

        self.declare_parameter('favorite_pose', [0.0, 0.0, 0.0])
        self.favorite_pose: List[float] = list(self.get_parameter('favorite_pose').value)

        self._status_pub = self.create_publisher(
            String,
            'navigation/status',
            10,
        )
        self.create_subscription(
            String,
            'navigation/goal',
            self._on_goal,
            10,
        )
        self.create_subscription(
            String,
            'navigation/cancel',
            self._on_cancel,
            10,
        )
        self._commander = CubePetitNavigationCommander(self)
        self._current_status: str = 'idle'
        self.create_timer(0.2, self._on_timer)
        self._publish_status('idle')
        self.get_logger().info('navigation_api_node started')

    def _on_goal(self, msg: String) -> None:
        """Handle incoming navigation goal requests."""
        text: str = msg.data.strip()
        self.get_logger().info(f'Received navigation goal: {text}')

        if self._commander.is_navigating():
            self.get_logger().info('Navigation already in progress, ignoring goal')
            return

        if text == 'favorite':
            pose = self._pose_from_list(self.favorite_pose)
            self._start_navigation(pose)
        elif text.startswith('pose:'):
            pose = self._parse_pose(text)
            self._start_navigation(pose)
        else:
            self.get_logger().warning(f'Unknown navigation goal: {text}')

    def _on_cancel(self, _msg: String) -> None:
        """Cancel the current navigation request."""
        self.get_logger().info('Cancel navigation request received')
        self._commander.cancel()
        self._publish_status('idle')

    def _on_timer(self) -> None:
        """Periodic check of navigation result."""
        result = self._commander.get_result()
        if result is True:
            self._publish_status('arrived')
        elif result is False:
            self._publish_status('failed')

    def _start_navigation(self, pose: PoseStamped) -> None:
        """Start navigation to the given pose."""
        self._commander.go_to_pose(pose)
        self._publish_status('navigating')

    def _publish_status(self, status: str) -> None:
        """Publish navigation status if it has changed."""
        if status == self._current_status:
            return

        self._current_status = status
        msg = String()
        msg.data = status
        self._status_pub.publish(msg)
        self.get_logger().info(f'Navigation status: {status}')

    def _pose_from_list(self, values: List[float]) -> PoseStamped:
        """Create a PoseStamped from [x, y, yaw]."""
        x, y, yaw = values
        return self._make_pose(x, y, yaw)

    def _parse_pose(self, text: str) -> PoseStamped:
        """Parse pose string formatted as 'pose:x,y,yaw'."""
        _, body = text.split(':', 1)
        x, y, yaw = (float(v) for v in body.split(','))
        return self._make_pose(x, y, yaw)

    def _make_pose(self, x: float, y: float, yaw: float) -> PoseStamped:
        """Construct a PoseStamped in the map frame."""
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()

        pose.pose.position.x = x
        pose.pose.position.y = y

        qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, yaw)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        return pose


def main() -> None:
    rclpy.init()
    node = NavigationApiNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
