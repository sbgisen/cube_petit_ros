#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright (c) 2021 SoftBank Corp.
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
"""ROS Node for switching facial expression depending on the robot's battery status."""

import sys

import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from rclpy.time_source import ClockType

from cube_petit_facial_animation_msgs.msg import FaceExpression

TOPIC_FACE = 'current_expression'
TOPIC_FACE_COMMAND = 'facial_expression/expression_command'
PARAM_OPERATION_BUFFER = 'param_operation_buffer_time'
PARAM_OPERATION_HZ = 'param_operation_hz'


class ExpressionOperator(Node):
    """A ROS Node content for automatically publishing a topic to change expression when battery level changes.

    A topic will not be published if this node or another node has published a topic within the last
    {buffer_time} seconds.
    """

    def __init__(self, node_name: str) -> None:
        """Init expression operator node."""
        super().__init__(node_name=node_name)
        self.declare_parameters(namespace='', parameters=[(PARAM_OPERATION_BUFFER, 30), (PARAM_OPERATION_HZ, 1.0)])
        # When expression is changed by other nodes, expression will not be changed by this node for this seconds.
        self.__buffer_time = self.get_parameter(PARAM_OPERATION_BUFFER).get_parameter_value().integer_value
        # The ros time when the expression was last changed.
        self.__changed_time = Time(clock_type=ClockType.ROS_TIME)
        # The above value is updated when this subscriber receives a message.
        self.create_subscription(FaceExpression, TOPIC_FACE_COMMAND, self.__cmd_callback, 10)
        # Float value representing the ratio of the battery remaining.

        # Publishes expression change command to the face node.
        self.__set_expression_pub = self.create_publisher(FaceExpression, TOPIC_FACE_COMMAND, 10)
        # Try to change expression every 1. / OPERATING_HZ sec.

    def __cmd_callback(self, _) -> None:
        """Update changed_time value when command to face_node (including commands from this node) is received."""
        self.__changed_time = self.get_clock().now()

    def switch_expression(self, expression: str) -> None:
        """Receive FaceExpression msg content and publish command msg if buffer time has passed.

        Args:
            expression (str): Name of expression to change to. Strings are defined inside FaceExpression type msg.
        """
        if self.get_clock().now() - self.__changed_time > Duration(seconds=self.__buffer_time):
            self.get_logger().debug(f'Sending expression command: {expression}')
            msg = FaceExpression()
            msg.expression = expression
            self.__set_expression_pub.publish(msg)


def main() -> None:
    """Run node."""
    rclpy.init()
    try:
        node = ExpressionOperator('expression_operator')
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)
    finally:
        rclpy.try_shutdown()
        node.destroy_node()


if __name__ == '__main__':
    main()
