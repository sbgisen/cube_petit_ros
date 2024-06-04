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
import sys
import time

import rclpy
# from rclpy.node import Node
import rclpy.node
from rclpy.action import ActionClient
# from rclpy.task import Future
from rclpy.executors import ExternalShutdownException

from sensor_msgs.msg import Joy
from sbgisen_msgs.action import Speech


class TextToJtalk(rclpy.node.Node):
    def __init__(self):
        super().__init__('cube_petit_text_to_jtalk')

        self.declare_parameter('my_parameter', 'world')

        self.timer = self.create_timer(3, self.timer_callback)

        self.__action_client = ActionClient(self, Speech, '/speech_action_server')
        self._joy_subscription = self.create_subscription(Joy,'joy', self.joystick_callback, 1)
        self.send_talk("起動しました！")

    def send_talk(self, talk_text:str):
        talk_msg = Speech.Goal()
        talk_msg.text = talk_text
        talk_msg.method = "jtalk"
        talk_msg.emotion = "happiness"
        talk_msg.emotion_level=  2
        talk_msg.pitch = 130
        talk_msg.speed = 100
        talk_msg.volume = 30

        self.__action_client.wait_for_server()
        self.get_logger().info("Robot say: [%s]" % (talk_text))
        return self.__action_client.send_goal_async(talk_msg)

    def send_talk_sad(self, talk_text:str):
        talk_msg = Speech.Goal()
        talk_msg.text = talk_text
        talk_msg.method = "jtalk"
        talk_msg.emotion = "sadness"
        talk_msg.emotion_level=  2
        talk_msg.pitch = 100
        talk_msg.speed = 100
        talk_msg.volume = 30

        self.__action_client.wait_for_server()
        self.get_logger().info("Robot say: [%s]" % (talk_text))
        return self.__action_client.send_goal_async(talk_msg)

    def send_talk_shout(self, talk_text:str):
        talk_msg = Speech.Goal()
        talk_msg.text = talk_text
        talk_msg.method = "jtalk"
        talk_msg.emotion = "shout"
        talk_msg.emotion_level=  2
        talk_msg.pitch = 100
        talk_msg.speed = 100
        talk_msg.volume = 30

        self.__action_client.wait_for_server()
        self.get_logger().info("Robot say: [%s]" % (talk_text))
        return self.__action_client.send_goal_async(talk_msg)

    def timer_callback(self):
        my_param = self.get_parameter('my_parameter').get_parameter_value().string_value

        self.get_logger().info('Hello %s!' % my_param)

        my_new_param = rclpy.parameter.Parameter(
            'my_parameter',
            rclpy.Parameter.Type.STRING,
            'world'
        )
        all_new_parameters = [my_new_param]
        self.set_parameters(all_new_parameters)

    def joystick_callback(self, Joy):
        if Joy.buttons[0] == 1:
            self.get_logger().info('x')
        elif Joy.buttons[1] == 1:
            self.get_logger().info('o')
        elif Joy.buttons[2] == 1:
            self.get_logger().info('△')
            self.send_talk("こんにちは")
        elif Joy.buttons[3] == 1:
            self.get_logger().info('□')
            self.send_talk("ハローワールド！僕の名前はキューブプチです！")
        elif Joy.buttons[4] == 1:
            self.get_logger().info('L1')
        elif Joy.buttons[5] == 1:
            self.get_logger().info('R1')
        elif Joy.buttons[6] == 1:
            self.get_logger().info('L2')
        elif Joy.buttons[7] == 1:
            self.get_logger().info('R2')
        elif Joy.buttons[8] == 1:
            self.get_logger().info('Select')
        elif Joy.buttons[9] == 1:
            self.get_logger().info('Start')
        elif Joy.buttons[10] == 1:
            self.get_logger().info('PS')
        elif Joy.buttons[11] == 1:
            self.get_logger().info('12')
        elif Joy.axes[6] == 1.0:
            self.get_logger().info('hidari')
        elif Joy.axes[6] == -1.0:
            self.get_logger().info('migi')
        elif Joy.axes[7] == 1.0:
            self.get_logger().info('ue')
        elif Joy.axes[7] == -1.0:
            self.get_logger().info('shita')
        time.sleep(1)

def main():
    rclpy.init()
    try:
        node = TextToJtalk()
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
