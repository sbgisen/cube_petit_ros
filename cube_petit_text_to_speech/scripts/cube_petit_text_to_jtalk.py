#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import random
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
import rclpy.node
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from sbgisen_msgs.action import Speech
from sensor_msgs.msg import Joy
from std_msgs.msg import String


class TextToJtalk(rclpy.node.Node):
    def __init__(self):
        super().__init__('cube_petit_text_to_jtalk')
        self.__action_client = ActionClient(self, Speech, '/speech_action_server')
        self._hand_gesture_subscription = self.create_subscription(
            String, '/hand_gesture', self.handgesture_callback, 1)
        self._hotword_subscription = self.create_subscription(String, '/detect_word', self.hotword_callback, 1)
        self._joy_subscription = self.create_subscription(Joy, 'joy', self.joystick_callback, 1)
        self.hand_gesture = None
        self.hand_gesture_received = False
        self.janken_flag = False
        self.timer = None
        self.robot_hand = None
        self.send_talk("起動しました！")

    def send_talk(self, talk_text: str):
        talk_msg = Speech.Goal()
        talk_msg.text = talk_text
        talk_msg.method = "jtalk"
        talk_msg.emotion = "happiness"
        talk_msg.emotion_level = 2
        talk_msg.pitch = 130
        talk_msg.speed = 100
        talk_msg.volume = 30

        self.__action_client.wait_for_server()
        self.get_logger().info("Robot say: [%s]" % (talk_text))
        return self.__action_client.send_goal_async(talk_msg)

    def decide_robot_hand(self) -> str:
        self.robot_hand = random.choice(["rock", "sissors", "paper"])
        if self.robot_hand == "rock":
            self.send_talk("ぐー")
            return "rock"
        elif self.robot_hand == "sissors":
            self.send_talk("ちょき")
            return "sissors"
        elif self.robot_hand == "paper":
            self.send_talk("ぱー")
            return "paper"

    def determine_winner(self):
        if self.robot_hand == self.hand_gesture:
            self.send_talk("引き分けです")
        elif (self.robot_hand == "rock" and self.hand_gesture == "sissors") or \
             (self.robot_hand == "sissors" and self.hand_gesture == "paper") or \
             (self.robot_hand == "paper" and self.hand_gesture == "rock"):
            self.send_talk("僕の勝ちです")
        else:
            self.send_talk("あなたの勝ちです")

    def hotword_callback(self, Hotword):
        if Hotword.data == "Cube-petit":
            self.send_talk("はーい")

    def handgesture_callback(self, HandGesture):
        if self.janken_flag:
            if HandGesture.data == "FIVE":
                self.send_talk("ぽん")
                self.decide_robot_hand()
                self.hand_gesture = "paper"
                self.get_logger().info("Received paper gesture")
                self.send_talk("ぱーを出したね")
                self.determine_winner()
                self.send_talk("また遊んでね")
                self.janken_flag = False
            elif HandGesture.data == "PEACE":
                self.send_talk("ぽん")
                self.decide_robot_hand()
                self.hand_gesture = "sissors"
                self.get_logger().info("Received scissors gesture")
                self.send_talk("ちょきを出したね")
                self.determine_winner()
                self.send_talk("また遊んでね")
                self.janken_flag = False
            elif HandGesture.data == "FIST":
                self.send_talk("ぽん")
                self.decide_robot_hand()
                self.hand_gesture = "rock"
                self.get_logger().info("Received rock gesture")
                self.send_talk("ぐーを出したね")
                self.determine_winner()
                self.send_talk("また遊んでね")
                self.janken_flag = False

    def timeout_callback(self):
        if self.janken_flag:
            self.get_logger().info("Timed out waiting for hand gesture")
            self.send_talk("また遊んでね")
            self.janken_flag = False
            self.timer = None

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
        elif Joy.axes[6] == 1.0 and not self.janken_flag:
            self.get_logger().info('hidari')
            self.send_talk("僕とじゃんけんで遊びましょう、最初はグー。じゃんけん")
            if self.timer is not None:
                self.timer.cancel()
            self.timer = self.create_timer(30.0, self.timeout_callback)

            self.janken_flag = True
        elif Joy.axes[6] == -1.0:
            self.get_logger().info('migi')
        elif Joy.axes[7] == 1.0:
            self.get_logger().info('ue')
        elif Joy.axes[7] == -1.0:
            self.get_logger().info('shita')
        time.sleep(1.0)


def main():
    rclpy.init()
    node = TextToJtalk()

    try:
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
