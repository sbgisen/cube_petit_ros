#!/usr/bin/env python

# Copyright (c) 2026 SoftBank Corp.
# 
# <<licensetext>>

import rclpy
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus


import importlib
import rclpy
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus


def import_msg_type(type_str: str):
    # e.g. "sensor_msgs/msg/Image"
    module_name, class_name = type_str.rsplit('/', 1)
    module_name = module_name.replace('/', '.')
    module = importlib.import_module(module_name)
    return getattr(module, class_name)


class TopicWatchdog(Node):
    def __init__(self):
        super().__init__('topic_watchdog')

        self.declare_parameter('topic', '')
        self.declare_parameter('msg_type', '')
        self.declare_parameter('timeout', 1.0)

        self.topic = self.get_parameter('topic').value
        type_str = self.get_parameter('msg_type').value
        self.timeout = float(self.get_parameter('timeout').value)

        if not self.topic or not type_str:
            self.get_logger().error("Parameters 'topic' and 'msg_type' are required")
            raise SystemExit(2)

        msg_type = import_msg_type(type_str)

        self.last_msg_time = None

        self.create_subscription(
            msg_type,
            self.topic,
            self._callback,
            10
        )

        self.pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.create_timer(0.5, self.publish_diagnostic)

    def _callback(self, msg):
        self.last_msg_time = self.get_clock().now()

    def publish_diagnostic(self):
        status = DiagnosticStatus()
        status.name = self.topic

        now = self.get_clock().now()
        if self.last_msg_time is None:
            status.level = DiagnosticStatus.ERROR
            status.message = 'No message received yet'
        else:
            dt = (now - self.last_msg_time).nanoseconds * 1e-9
            if dt > self.timeout:
                status.level = DiagnosticStatus.ERROR
                status.message = f'Timeout ({dt:.2f}s)'
            else:
                status.level = DiagnosticStatus.OK
                status.message = 'OK'

        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.status.append(status)
        self.pub.publish(msg)


def main():
    rclpy.init()
    try:
        node = TopicWatchdog()
        rclpy.spin(node)
    except SystemExit as e:
        # Clean exit for missing parameter
        pass
