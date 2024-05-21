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

from typing import List

import rclpy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from std_msgs.msg import Header


class TwistToTwistStamped(Node):
    def __init__(self) -> None:
        super().__init__('twist_to_twist_stamped')
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.listener_callback,
            10)
        self.publisher_ = self.create_publisher(TwistStamped, '/diff_drive_controller/cmd_vel', 10)

    def listener_callback(self, msg: Twist) -> None:
        twist_stamped_msg = TwistStamped()
        twist_stamped_msg.header = Header()
        twist_stamped_msg.header.stamp = self.get_clock().now().to_msg()
        twist_stamped_msg.header.frame_id = 'base_link'
        twist_stamped_msg.twist = msg
        self.publisher_.publish(twist_stamped_msg)


def main(args: List[str] = None) -> None:
    rclpy.init(args=args)
    twist_to_twist_stamped = TwistToTwistStamped()
    rclpy.spin(twist_to_twist_stamped)

    twist_to_twist_stamped.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
