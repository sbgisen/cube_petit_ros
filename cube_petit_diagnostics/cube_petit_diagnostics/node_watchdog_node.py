#!/usr/bin/env python

# Copyright (c) 2026 SoftBank Corp.
# 
# <<licensetext>>

import rclpy
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus


class NodeWatchdog(Node):
    def __init__(self):
        super().__init__('node_watchdog')

        self.declare_parameter('node_name', '')
        self.declare_parameter('timeout', 2.0)

        self.node_name = self.get_parameter('node_name').value
        self.timeout = float(self.get_parameter('timeout').value)

        if not self.node_name:
            self.get_logger().error("Parameter 'node_name' is required")
            raise SystemExit(2)

        self.last_seen = None
        self.pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)

        self.create_timer(0.5, self.check)

    def check(self):
        nodes = self.get_node_names_and_namespaces()

        found = any(name == self.node_name for name, _ in nodes)
        if found:
            self.last_seen = self.get_clock().now()

        status = DiagnosticStatus()
        status.name = f'node/{self.node_name.lstrip("/")}'

        if self.last_seen is None:
            status.level = DiagnosticStatus.ERROR
            status.message = 'Node not found'
        else:
            dt = (self.get_clock().now() - self.last_seen).nanoseconds * 1e-9
            if dt > self.timeout:
                status.level = DiagnosticStatus.ERROR
                status.message = f'Node disappeared ({dt:.2f}s)'
            else:
                status.level = DiagnosticStatus.OK
                status.message = 'OK'

        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.status.append(status)
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = NodeWatchdog()
    rclpy.spin(node)
