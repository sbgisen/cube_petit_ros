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

import pathlib
from typing import Dict, List, Tuple

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
import rclpy.time
from std_msgs.msg import String
from tf2_ros import Buffer
from tf2_ros import TransformListener
from tf_transformations import quaternion_from_euler
import yaml

from cube_petit_navigation.cube_petit_patrol_commander import CubePetitPatrolCommander
from cube_petit_navigation.navigation.cube_petit_navigation_commander import CubePetitNavigationCommander
from cube_petit_navigation.patrol.patrol_controller import PatrolController
from cube_petit_navigation.places.places_store import PlacesStore
from cube_petit_navigation_msgs.msg import NavigationState
from cube_petit_navigation_msgs.srv import SavePlace


class NavigationApiNode(Node):
    """Navigation API node using external YAML config file."""

    def __init__(self) -> None:
        super().__init__('navigation_api_node')

        # ================= config file =================

        pkg_path = get_package_share_directory('cube_petit_navigation')
        self.declare_parameter(
            'places_config_file',
            str(pathlib.Path(pkg_path) / 'config' / 'places.yaml'),
        )
        places_path = pkg_path / 'config' / 'places.yaml'
        config_path = pathlib.Path(self.get_parameter('places_config_file').value)
        self._places_store = PlacesStore(places_path)
        self.get_logger().info(f'Loading places config: {config_path}')

        with config_path.open() as f:
            cfg = yaml.safe_load(f)

        self._patrol_cfg: Dict = cfg.get('patrol', {})
        self._favorite_cfg: Dict = cfg.get('favorite', {})
        self._dock_cfg: Dict = cfg.get('dock', {})

        self._rooms: Dict[str, List[Tuple[float, float]]] = {}
        for name, room in cfg.get('rooms', {}).items():
            self._rooms[name] = [(p['x'], p['y']) for p in room.get('points', [])]

        self.get_logger().info(f'Loaded rooms: {list(self._rooms.keys())}')

        self._current_room: str | None = None
        self._current_status: str = 'idle'

        self.create_service(
            SavePlace,
            'navigation/save_place',
            self._on_save_place,
        )
        self.create_service(
            NavigationState,
            'navigation/get_state',
            self._on_get_state,
        )

        # ================= TF =================

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # ================= publishers =================

        self._status_pub = self.create_publisher(String, 'navigation/status', 10)
        self._room_pub = self.create_publisher(String, 'navigation/room', 10)

        # ================= subscribers =================

        self.create_subscription(String, 'navigation/goal', self._on_goal, 10)
        self.create_subscription(String, 'navigation/cancel', self._on_cancel, 10)

        # ================= commanders =================

        self._nav_commander = CubePetitNavigationCommander(self)
        self._patrol_commander = CubePetitPatrolCommander(self)

        self._patrol_controller = PatrolController(
            node=self,
            patrol_cfg=self._patrol_cfg,
            patrol_commander=self._patrol_commander,
        )

        # ================= timer =================

        self.create_timer(0.2, self._on_timer)

        self._publish_status('idle')
        self.get_logger().info('navigation_api_node started')

    # =================================================
    # Callbacks
    # =================================================
    def _on_get_state(self, _, res: NavigationState) -> NavigationState:
        res.status = self._current_status
        res.room = self._current_room or 'unknown'
        return res

    def _on_save_place(
        self,
        request: SavePlace.Request,
        response: SavePlace.Response,
    ) -> SavePlace.Response:
        pos = self._get_current_xy()
        if pos is None:
            response.success = False
            response.message = 'Current position unavailable'
            return response

        x, y = pos
        yaw = 0.0  # [TODO]

        room = self._detect_room(x, y)

        self._places_store.save_place(
            category=request.category,
            name=request.name,
            pose=[x, y, yaw],
            room=room,
        )

        response.success = True
        response.message = f'Saved place "{request.name}"'
        self.get_logger().info(response.message)
        return response

    def _on_goal(self, msg: String) -> None:
        text = msg.data.strip()
        self.get_logger().info(f'Received navigation goal: {text}')

        if text.startswith('pose:'):
            # pose: goals are allowed to preempt an in-progress navigation or
            # patrol, unlike favorite/patrol below. This is needed for use
            # cases like chasing another robot's live position, where the
            # target is re-sent frequently and each update must take over
            # immediately instead of being silently dropped because a
            # (now-stale) previous pose goal was still "running".
            if self._patrol_controller.is_running():
                self._patrol_controller.cancel()
            pose = self._parse_pose(text)
            self._start_navigation(pose)
            return

        if self._nav_commander.is_navigating() or self._patrol_controller.is_running():
            self.get_logger().info('Navigation already running')
            return

        if text == 'favorite':
            pose = self._get_favorite_pose()
            if pose:
                self._start_navigation(pose)

        elif text == 'patrol':
            self._patrol_controller.start()
            self._publish_status('patrolling')

        else:
            self.get_logger().warning(f'Unknown goal: {text}')

    def _on_cancel(self, _msg: String) -> None:
        self._nav_commander.cancel()
        self._patrol_controller.cancel()
        self._publish_status('idle')

    def _on_timer(self) -> None:
        # navigation result
        result = self._nav_commander.get_result()
        if result is True:
            self._publish_status('arrived')
        elif result is False:
            self._publish_status('failed')

        # patrol update
        self._patrol_controller.update()

        # room detection
        pos = self._get_current_xy()
        if pos:
            room = self._detect_room(*pos)
            if room != self._current_room:
                self._current_room = room
                self._publish_room(room)

    # =================================================
    # Helpers
    # =================================================

    def _get_current_xy(self) -> Tuple[float, float] | None:
        try:
            tf = self._tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time(),
                timeout=Duration(seconds=0.2),
            )
            return tf.transform.translation.x, tf.transform.translation.y
        except Exception:
            return None

    def _detect_room(self, x: float, y: float) -> str | None:
        for name, poly in self._rooms.items():
            if self._point_in_polygon(x, y, poly):
                return name
        return None

    @staticmethod
    def _point_in_polygon(
        x: float,
        y: float,
        polygon: List[Tuple[float, float]],
    ) -> bool:
        inside = False
        n = len(polygon)
        for i in range(n):
            x1, y1 = polygon[i]
            x2, y2 = polygon[(i + 1) % n]
            if ((y1 > y) != (y2 > y)) and \
               (x < (x2 - x1) * (y - y1) / (y2 - y1 + 1e-9) + x1):
                inside = not inside
        return inside

    def _get_favorite_pose(self) -> PoseStamped | None:
        order = self._favorite_cfg.get('order', [])
        if not order:
            return None
        entry = self._favorite_cfg['places'].get(order[0])
        if not entry:
            return None
        x, y, yaw = entry['pose']
        return self._make_pose(x, y, yaw)

    def _parse_pose(self, text: str) -> PoseStamped:
        _, body = text.split(':', 1)
        x, y, yaw = map(float, body.split(','))
        return self._make_pose(x, y, yaw)

    def _make_pose(self, x: float, y: float, yaw: float) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        qx, qy, qz, qw = quaternion_from_euler(0, 0, yaw)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        return pose

    def _start_navigation(self, pose: PoseStamped) -> None:
        self._nav_commander.go_to_pose(pose)
        self._publish_status('navigating')

    def _publish_status(self, status: str) -> None:
        if status != self._current_status:
            self._current_status = status
            self._status_pub.publish(String(data=status))

    def _publish_room(self, room: str | None) -> None:
        self._room_pub.publish(String(data=room or 'unknown'))


def main() -> None:
    rclpy.init()
    node = NavigationApiNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
