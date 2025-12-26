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

from typing import Dict, List, Optional

from geometry_msgs.msg import PoseStamped
from rclpy.node import Node

from cube_petit_navigation.cube_petit_patrol_commander import CubePetitPatrolCommander


class PatrolController:
    """Controller class that manages patrol behavior using FollowWaypoints."""

    def __init__(
        self,
        node: Node,
        patrol_cfg: Dict,
        patrol_commander: CubePetitPatrolCommander,
    ) -> None:
        """
        Init.

        Args:
            node: ROS2 node instance.
            patrol_cfg: patrol section loaded from places.yaml.
            patrol_commander: Commander for FollowWaypoints action.
        """
        self._node: Node = node
        self._cfg: Dict = patrol_cfg
        self._commander: CubePetitPatrolCommander = patrol_commander

        self._poses: List[PoseStamped] = []
        self._active: bool = False

        self._load_patrol_poses()

    # =================================================
    # Public API
    # =================================================

    def start(self) -> None:
        """Start patrol."""
        if not self._poses:
            self._node.get_logger().warning('No patrol poses available')
            return

        if self._active:
            self._node.get_logger().info('Patrol already running')
            return

        self._node.get_logger().info('Starting patrol')
        self._commander.start_patrol(self._poses)
        self._active = True

    def cancel(self) -> None:
        """Cancel patrol."""
        if not self._active:
            return

        self._node.get_logger().info('Canceling patrol')
        self._commander.cancel()
        self._active = False

    def is_running(self) -> bool:
        """Return True if patrol is active."""
        return self._active

    def update(self) -> None:
        """Update patrol state. Call periodically from a timer."""
        if not self._active:
            return

        result: Optional[bool] = self._commander.get_result()
        if result is None:
            return

        if result is True:
            self._node.get_logger().info('Patrol finished successfully')
        else:
            self._node.get_logger().warning('Patrol failed')

        self._active = False

    # =================================================
    # Internal helpers
    # =================================================

    def _load_patrol_poses(self) -> None:
        """Load patrol poses from configuration."""
        order = self._cfg.get('order', [])
        places = self._cfg.get('places', {})
        if not order:
            self._node.get_logger().warning('Patrol order is empty')
            return

        for name in order:
            entry = places.get(name)
            if entry is None:
                self._node.get_logger().warning(f'Patrol place "{name}" not found')
                continue

            pose_list = entry.get('pose')
            if pose_list is None or len(pose_list) != 3:
                self._node.get_logger().warning(f'Invalid pose for patrol place "{name}"')
                continue

            pose = self._make_pose(*pose_list)
            self._poses.append(pose)

        self._node.get_logger().info(f'Loaded {len(self._poses)} patrol waypoints',)

    def _make_pose(self, x: float, y: float, yaw: float) -> PoseStamped:
        """Create PoseStamped in map frame."""
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self._node.get_clock().now().to_msg()

        pose.pose.position.x = x
        pose.pose.position.y = y

        # yaw -> quaternion
        from tf_transformations import quaternion_from_euler

        qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, yaw)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        return pose
