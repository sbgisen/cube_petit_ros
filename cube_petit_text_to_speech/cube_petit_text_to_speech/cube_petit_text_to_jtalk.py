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

import random
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
import rclpy.node
from cube_petit_speech_msgs.action import Speech
from sensor_msgs.msg import Joy
from std_msgs.msg import String
from typing import Any, Dict

import rclpy
import rclpy.node
import yaml

class TextToJtalk(rclpy.node.Node):

    def __init__(self) -> None:
        super().__init__('cube_petit_text_to_jtalk')

        self._hand_gesture_subscription = self.create_subscription(String, 'hand_gesture', self.handgesture_callback,
                                                                   1)
        self._hotword_subscription = self.create_subscription(String, 'detect_word', self.hotword_callback, 1)
        self._joy_subscription = self.create_subscription(Joy, 'diff_drive_controller/joy', self.joystick_callback, 1)
        self.hand_gesture = None
        self.hand_gesture_received = False
        self.janken_flag = False
        self.timer = None
        self.robot_hand = None

        self.__action_client = ActionClient(self, Speech, 'speech_action_server')
        self.__action_client.wait_for_server()

        self.declare_parameter("controller_talk_config", "")
        path = self.get_parameter("controller_talk_config").value

        with open(path, "r", encoding="utf-8") as f:
            cfg = yaml.safe_load(f)

        self.buttons_map = cfg.get("buttons", {})
        self.axes_map = cfg.get("axes", {})
        self.debounce_sec = float(cfg.get("debounce_sec", 1.0))

        self.get_logger().info(f'Loaded buttons_map: {self.buttons_map}')
        self.get_logger().info(f'Loaded axes_map: {self.axes_map}')
        self.get_logger().info(f'Loaded debounce_sec: {self.debounce_sec}')

        self._goal_handle = None
        self.hand_gesture_received = False

    def send_talk(self, talk_text: str) -> None:
        self.cancel_talk()

        talk_msg = Speech.Goal()
        talk_msg.text = talk_text
        talk_msg.emotion = 'happy'
        talk_msg.emotion_level = 2
        talk_msg.pitch = 130
        talk_msg.speed = 100
        talk_msg.volume = 100

        self.get_logger().info(f'Robot say: [{talk_text}]')

        future = self.__action_client.send_goal_async(talk_msg)
        future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Speech goal rejected")
            return

        self.get_logger().info("Speech goal accepted")
        self._goal_handle = goal_handle

    def cancel_talk(self) -> None:
        if self._goal_handle is None:
            self.get_logger().info("No active goal to cancel")
            return

        self.get_logger().info("Cancel request sent")
        cancel_future = self._goal_handle.cancel_goal_async()
        cancel_future.add_done_callback(self._cancel_done_callback)

    def _cancel_done_callback(self, future):
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info("Speech goal canceled")
        else:
            self.get_logger().warn("Speech goal cancel failed")

    def decide_robot_hand(self) -> str:
        self.robot_hand = random.choice(['rock', 'sissors', 'paper'])
        if self.robot_hand == 'rock':
            self.send_talk('ぐー')
            return 'rock'
        elif self.robot_hand == 'sissors':
            self.send_talk('ちょき')
            return 'sissors'
        elif self.robot_hand == 'paper':
            self.send_talk('ぱー')
            return 'paper'

    def determine_winner(self) -> None:
        if self.robot_hand == self.hand_gesture:
            self.send_talk('引き分けです')
        elif (self.robot_hand == 'rock' and self.hand_gesture == 'sissors') or \
             (self.robot_hand == 'sissors' and self.hand_gesture == 'paper') or \
             (self.robot_hand == 'paper' and self.hand_gesture == 'rock'):
            self.send_talk('僕の勝ちです')
        else:
            self.send_talk('あなたの勝ちです')

    def hotword_callback(self, hotword: String) -> None:
        if hotword.data == 'Cube-petit':
            self.send_talk('はーい')

    def handgesture_callback(self, hand_gesture: String) -> None:
        self.hand_gesture_received = True
        if self.janken_flag:
            if hand_gesture.data == 'FIVE':
                self.send_talk('ぽん')
                self.decide_robot_hand()
                self.hand_gesture = 'paper'
                self.get_logger().info('Received paper gesture')
                self.send_talk('ぱーを出したね')
                self.determine_winner()
                self.send_talk('また遊んでね')
                self.janken_flag = False
            elif hand_gesture.data == 'PEACE':
                self.send_talk('ぽん')
                self.decide_robot_hand()
                self.hand_gesture = 'sissors'
                self.get_logger().info('Received scissors gesture')
                self.send_talk('ちょきを出したね')
                self.determine_winner()
                self.send_talk('また遊んでね')
                self.janken_flag = False
            elif hand_gesture.data == 'FIST':
                self.send_talk('ぽん')
                self.decide_robot_hand()
                self.hand_gesture = 'rock'
                self.get_logger().info('Received rock gesture')
                self.send_talk('ぐーを出したね')
                self.determine_winner()
                self.send_talk('また遊んでね')
                self.janken_flag = False

    def timeout_callback(self) -> None:
        if self.janken_flag:
            self.get_logger().info('Timed out waiting for hand gesture')
            self.send_talk('また遊んでね')
            self.janken_flag = False
            self.timer = None

    def _execute_action(self, action: Dict[str, Any]) -> None:
        """Execute one YAML-defined action dict."""
        if not isinstance(action, dict):
            return

        log_text = action.get('log', None)
        talk_text = action.get('talk', None)

        if log_text:
            self.get_logger().info(str(log_text))

        if talk_text:
            self.send_talk(str(talk_text))

        janken = action.get('janken', None)
        if isinstance(janken, dict):
            enable_flag = bool(janken.get('enable_flag', False))
            timeout_sec = float(janken.get('timeout_sec', 30.0))

            if enable_flag and not self.janken_flag:
                if not self.hand_gesture_received:
                    self.send_talk("手がまだ見えてないよ。もう一度ジェスチャーしてね。")
                    return

                # start janken
                if self.timer is not None:
                    self.timer.cancel()
                self.timer = self.create_timer(timeout_sec, self.timeout_callback)
                self.janken_flag = True

    def joystick_callback(self, joy: Joy) -> None:
        for idx, val in enumerate(joy.buttons):
            if val != 1:
                continue
            key = str(idx)
            action = self.buttons_map.get(key, None)
            if action:
                self._execute_action(action)
                time.sleep(self.debounce_sec)
                return
        for axis_idx, axis_val in enumerate(joy.axes):
            axis_key = str(axis_idx)
            axis_table = self.axes_map.get(axis_key, None)
            if not isinstance(axis_table, dict):
                continue
            v = round(float(axis_val), 1)
            value_key = str(v)

            action = axis_table.get(value_key, None)
            if action:
                self._execute_action(action)
                time.sleep(self.debounce_sec)
                return


def main() -> None:
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
