#!/usr/bin/env python3
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

import threading
import time

from controller_manager_msgs.srv import ListControllerTypes
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

from cube_petit_speech_msgs.action import Speech


class StartupAnnouncer(Node):

    def __init__(self) -> None:
        super().__init__('startup_announcer')

        # ===== parameters =====
        self.declare_parameter('text', '起動しました。')
        self.declare_parameter('wait_sec', 3.0)
        self.declare_parameter('controller_check_timeout_sec', 10.0)
        self.controller_check_timeout_sec = float(self.get_parameter('controller_check_timeout_sec').value)

        self.declare_parameter('required_nodes', [
            'speech_action_server',
            'text_to_jtalk',
            'ldlidar_publisher_ld06',
        ])

        self.declare_parameter('required_scan_topic', 'scan')

        self.declare_parameter('node_check_timeout_sec', 10.0)
        self.declare_parameter('topic_check_timeout_sec', 10.0)

        self.declare_parameter('required_controller_types', [
            'joint_state_broadcaster/JointStateBroadcaster',
            'diff_drive_controller/DiffDriveController',
        ])
        self.required_controller_types = list(self.get_parameter('required_controller_types').value)

        self.declare_parameter('controller_grace_sec', 10.0)
        self.controller_grace_sec = float(self.get_parameter('controller_grace_sec').value)
        self.declare_parameter('controller_manager_service', 'controller_manager')
        self.controller_manager_service = self.get_parameter('controller_manager_service').value

        self.text = self.get_parameter('text').value
        self.wait_sec = float(self.get_parameter('wait_sec').value)
        self.required_nodes = list(self.get_parameter('required_nodes').value)
        self.required_scan_topic = self.get_parameter('required_scan_topic').value
        self.node_check_timeout_sec = float(self.get_parameter('node_check_timeout_sec').value)
        self.topic_check_timeout_sec = float(self.get_parameter('topic_check_timeout_sec').value)

        # ===== action client =====
        self._client = ActionClient(self, Speech, 'speech_action_server')

        # ===== topic check =====
        self._scan_received = False
        self._scan_sub = self.create_subscription(LaserScan, self.required_scan_topic, self._scan_callback, 10)

        # ===== start =====
        self._started = False
        self._timer = self.create_timer(0.1, self._on_timer)

    # NOTE: The startup sequence below runs in a worker thread while the main
    # thread keeps spinning the node (see main()). Never call
    # rclpy.spin_once() from here: the main executor is already spinning and
    # a nested spin raises "Executor is already spinning" (issue #96).
    # Subscriptions and futures are serviced by the main-thread spin, so
    # plain sleeps + polling are enough.

    def _wait_controller_types(self) -> bool:
        service_name = f'{self.controller_manager_service}/list_controller_types'
        if not service_name.startswith('/'):
            service_name = '/' + service_name

        self.get_logger().info(f'Checking controller types service: {service_name}')

        client = self.create_client(ListControllerTypes, service_name)

        if not client.wait_for_service(timeout_sec=10.0):
            self.get_logger().warn(f'Service not available: {service_name}')
            return False

        start = time.time()
        while time.time() - start < self.controller_check_timeout_sec:
            req = ListControllerTypes.Request()
            future = client.call_async(req)

            # The main-thread spin completes the future; just poll it here.
            t0 = time.time()
            while not future.done() and time.time() - t0 < 2.0:
                time.sleep(0.1)

            if not future.done() or future.result() is None:
                self.get_logger().info('Waiting controller types...')
                time.sleep(0.2)
                continue

            res = future.result()

            available_list = []
            if hasattr(res, 'types'):
                available_list = list(res.types)
            elif hasattr(res, 'controller_types'):
                available_list = list(res.controller_types)

            if not available_list:
                self.get_logger().info('Controller types list is empty yet...')
                time.sleep(0.2)
                continue

            available = set(available_list)
            missing = [t for t in self.required_controller_types if t not in available]

            if not missing:
                self.get_logger().info('All required controller types are available.')
                return True

            self.get_logger().info(f'Missing controller types: {missing}')
            time.sleep(0.2)

        self.get_logger().warn('Controller type check timeout.')
        return False

    def _scan_callback(self, msg: LaserScan) -> None:
        self._scan_received = True

    def _wait_required_nodes(self) -> bool:
        """Wait until all required nodes are visible."""
        start = time.time()
        while time.time() - start < self.node_check_timeout_sec:
            visible = self.get_node_names_and_namespaces()
            visible_full = {f'{ns}/{name}' for (name, ns) in visible}

            missing = [n for n in self.required_nodes if n not in visible_full]
            if len(missing) == 0:
                self.get_logger().info('All required nodes are running.')
                return True

            self.get_logger().info(f'Waiting nodes... missing: {missing}')
            time.sleep(0.2)

        self.get_logger().warn('Node check timeout.')
        return False

    def _wait_scan_topic(self) -> bool:
        """Wait until /scan is actually received."""
        start = time.time()
        while time.time() - start < self.topic_check_timeout_sec:
            if self._scan_received:
                self.get_logger().info(f'Scan topic OK: {self.required_scan_topic}')
                return True
            time.sleep(0.2)

        self.get_logger().warn(f'Topic check timeout: {self.required_scan_topic}')
        return False

    def _on_timer(self) -> None:
        # Fires once after the executor has started spinning, then hands the
        # blocking startup sequence to a worker thread so the main-thread
        # spin keeps servicing callbacks and futures.
        if self._started:
            return
        self._started = True
        self._timer.cancel()
        threading.Thread(target=self._run_startup_sequence, daemon=True).start()

    def _run_startup_sequence(self) -> None:
        self.get_logger().info(f'Wait {self.wait_sec} sec before checks...')
        time.sleep(self.wait_sec)

        nodes_ok = self._wait_required_nodes()

        topic_ok = self._wait_scan_topic()

        self.get_logger().info(f'Grace wait {self.controller_grace_sec} sec for controller_manager...')
        time.sleep(self.controller_grace_sec)

        controllers_ok = self._wait_controller_types()

        if not nodes_ok or not topic_ok or not controllers_ok:
            self.get_logger().warn('Startup check failed. Not announcing.')
            rclpy.shutdown()
            return

        self.get_logger().info('Waiting for speech_action_server action...')
        if not self._client.wait_for_server(timeout_sec=15.0):
            self.get_logger().warn('speech_action_server action not available. Not announcing.')
            rclpy.shutdown()
            return

        goal = Speech.Goal()
        goal.text = str(self.text)
        goal.emotion = 'happy'
        goal.emotion_level = 2
        goal.pitch = 130
        goal.speed = 100
        goal.volume = 100

        self.get_logger().info(f'Announce: {self.text}')
        goal_future = self._client.send_goal_async(goal)

        # Make sure the goal actually reached the speech server before we
        # shut down; the server keeps playing the announcement on its own.
        t0 = time.time()
        while not goal_future.done() and time.time() - t0 < 5.0:
            time.sleep(0.1)
        if not goal_future.done():
            self.get_logger().warn('Speech goal delivery timed out.')

        rclpy.shutdown()


def main() -> None:
    rclpy.init()
    node = StartupAnnouncer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # Normal exit paths: Ctrl-C from the launch, or our own
        # rclpy.shutdown() issued by the worker thread when done.
        pass


if __name__ == '__main__':
    main()
