#!/usr/bin/env python

# Copyright (c) 2026 SoftBank Corp.
# 
# <<licensetext>>

#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from controller_manager_msgs.srv import ListControllerTypes

from cube_petit_speech_msgs.action import Speech
from sensor_msgs.msg import LaserScan

class StartupAnnouncer(Node):
    def __init__(self):
        super().__init__("startup_announcer")

        # ===== parameters =====
        self.declare_parameter("text", "起動しました。")
        self.declare_parameter("wait_sec", 3.0)
        self.declare_parameter("controller_check_timeout_sec", 10.0)
        self.controller_check_timeout_sec = float(self.get_parameter("controller_check_timeout_sec").value)

        self.declare_parameter("required_nodes", [
            "speech_action_server",
            "text_to_jtalk",
            "ldlidar_publisher_ld06",
        ])

        self.declare_parameter("required_scan_topic", "scan")

        self.declare_parameter("node_check_timeout_sec", 10.0)
        self.declare_parameter("topic_check_timeout_sec", 10.0)

        self.declare_parameter("required_controller_types", [
            "joint_state_broadcaster/JointStateBroadcaster",
            "diff_drive_controller/DiffDriveController",
        ])
        self.required_controller_types = list(self.get_parameter("required_controller_types").value)

        self.declare_parameter("controller_grace_sec", 10.0)
        self.controller_grace_sec = float(self.get_parameter("controller_grace_sec").value)
        self.declare_parameter("controller_manager_service", "controller_manager")
        self.controller_manager_service = self.get_parameter("controller_manager_service").value


        self.text = self.get_parameter("text").value
        self.wait_sec = float(self.get_parameter("wait_sec").value)
        self.required_nodes = list(self.get_parameter("required_nodes").value)
        self.required_scan_topic = self.get_parameter("required_scan_topic").value
        self.node_check_timeout_sec = float(self.get_parameter("node_check_timeout_sec").value)
        self.topic_check_timeout_sec = float(self.get_parameter("topic_check_timeout_sec").value)

        # ===== action client =====
        self._client = ActionClient(self, Speech, "speech_action_server")

        # ===== topic check =====
        self._scan_received = False
        self._scan_sub = self.create_subscription(
            LaserScan,
            self.required_scan_topic,
            self._scan_callback,
            10
        )

        # ===== start =====
        self._started = False
        self._timer = self.create_timer(0.1, self._on_timer)

    def _sleep_with_spin(self, sec: float):
        end = time.time() + sec
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.1)

    def _wait_controller_types(self) -> bool:
        service_name = f"{self.controller_manager_service}/list_controller_types"
        if not service_name.startswith("/"):
            service_name = "/" + service_name

        self.get_logger().info(f"Checking controller types service: {service_name}")

        client = self.create_client(ListControllerTypes, service_name)

        if not client.wait_for_service(timeout_sec=10.0):
            self.get_logger().warn(f"Service not available: {service_name}")
            return False

        start = time.time()
        while time.time() - start < self.controller_check_timeout_sec:
            req = ListControllerTypes.Request()
            future = client.call_async(req)

            # ★ここがポイント：spin_onceでfutureが終わるのを待つ
            t0 = time.time()
            while not future.done() and time.time() - t0 < 2.0:
                rclpy.spin_once(self, timeout_sec=0.1)

            if not future.done() or future.result() is None:
                self.get_logger().info("Waiting controller types...")
                time.sleep(0.2)
                continue

            res = future.result()

            available_list = []
            if hasattr(res, "types"):
                available_list = list(res.types)
            elif hasattr(res, "controller_types"):
                available_list = list(res.controller_types)

            if not available_list:
                self.get_logger().info("Controller types list is empty yet...")
                time.sleep(0.2)
                continue

            available = set(available_list)
            missing = [t for t in self.required_controller_types if t not in available]

            if not missing:
                self.get_logger().info("All required controller types are available.")
                return True

            self.get_logger().info(f"Missing controller types: {missing}")
            time.sleep(0.2)

        self.get_logger().warn("Controller type check timeout.")
        return False


    def _scan_callback(self, msg: LaserScan):
        self._scan_received = True

    def _wait_required_nodes(self) -> bool:
        """Wait until all required nodes are visible."""
        start = time.time()
        while time.time() - start < self.node_check_timeout_sec:
            visible = self.get_node_names_and_namespaces()
            visible_full = set([f"{ns}/{name}" for (name, ns) in visible])

            missing = [n for n in self.required_nodes if n not in visible_full]
            if len(missing) == 0:
                self.get_logger().info("All required nodes are running.")
                return True

            self.get_logger().info(f"Waiting nodes... missing: {missing}")
            rclpy.spin_once(self, timeout_sec=0.2)

        self.get_logger().warn("Node check timeout.")
        return False

    def _wait_scan_topic(self) -> bool:
        """Wait until /scan is actually received."""
        start = time.time()
        while time.time() - start < self.topic_check_timeout_sec:
            if self._scan_received:
                self.get_logger().info(f"Scan topic OK: {self.required_scan_topic}")
                return True
            rclpy.spin_once(self, timeout_sec=0.2)

        self.get_logger().warn(f"Topic check timeout: {self.required_scan_topic}")
        return False

    def _on_timer(self):
        if self._started:
            return
        self._started = True

        self.get_logger().info(f"Wait {self.wait_sec} sec before checks...")
        self._sleep_with_spin(self.wait_sec)

        nodes_ok = self._wait_required_nodes()

        topic_ok = self._wait_scan_topic()

        self.get_logger().info(f"Grace wait {self.controller_grace_sec} sec for controller_manager...")
        self._sleep_with_spin(self.controller_grace_sec)
                
        controllers_ok = self._wait_controller_types()

        if not nodes_ok or not topic_ok or not controllers_ok:
            self.get_logger().warn("Startup check failed. Not announcing.")
            rclpy.shutdown()
            return


        self.get_logger().info("Waiting for speech_action_server action...")
        self._client.wait_for_server()

        goal = Speech.Goal()
        goal.text = str(self.text)
        goal.emotion = "happy"
        goal.emotion_level = 2
        goal.pitch = 130
        goal.speed = 100
        goal.volume = 100

        self.get_logger().info(f"Announce: {self.text}")
        self._client.send_goal_async(goal)

        rclpy.shutdown()


def main():
    rclpy.init()
    node = StartupAnnouncer()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
