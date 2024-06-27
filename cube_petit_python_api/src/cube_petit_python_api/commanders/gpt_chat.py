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
"""GPT Chat"""

import base64
import json
import os

import cv2
import np
import openai
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.duration import Duration
from rclpy.node import Node
from std_msgs.msg import String

from cube_petit_python_api.commanders.speech import SpeechCommander
from cube_petit_python_api.utils.gpt_client import GPTClient


class GPTChatCommander:
    """GPTChatCommander."""

    def __init__(self, node: Node) -> None:
        """Constructor."""
        self.api_key = os.getenv('OPENAI_API_KEY', '')
        self.node = node

        if self.api_key != '':
            self.node.get_logger().info("Set OPENAI_API_KEY")
        else:
            self.node.get_logger().warn("No OPENAI_API_KEY in ENV")

        package_path = get_package_share_directory('cube_petit_python_api')
        default_setting_file_path = f"{package_path}/config/cube_petit_gpt_setting.txt"

        self.__setting_file = self.node.declare_parameter('setting_file',
                                                          default_setting_file_path).get_parameter_value().string_value
        try:
            with open(self.__setting_file, 'r') as file:
                self.__setting_file__text = file.read()
        except FileNotFoundError:
            self.node.get_logger().error(f"設定ファイル {self.__setting_file} が見つかりません。")
            return None
        except Exception as e:
            self.node.get_logger().error(f"設定ファイルの読み込み中にエラーが発生しました: {e}")
            return None

        self.node.get_logger().info(self.__setting_file)
        self.__julius_text = None
        self.node.create_subscription(String, '/julius_talk_result', self.__julius_callback, 1)
        self.__speaker = GPTClient(self.node,
                                   api_key=self.api_key,
                                   setting_file=self.__setting_file)
        self.__chat_history = [{"role": "system", "content": self.__setting_file__text}]
        self.__max_jtalk_retry_time = 3
        self.timeout_duration = 30

    def __julius_callback(self, result_text: String) -> None:
        self.__julius_text = result_text.data
        self.node.get_logger().debug("__julius_callback")
        self.node.get_logger().debug(self.__julius_text)
        self.node.get_logger().debug("-----------------")

    def chat(self, input_robot_text: str, timeout: int = 30, image: np.darray = None) -> str:
        """Chat."""
        self.node.get_logger().info('Waiting for chat')
        speech_commander = SpeechCommander(self.node)
        if timeout is not None:
            self.timeout_duration = Duration(seconds=timeout)
        self.__chat_history.append({"role": "assistant", "content": input_robot_text})
        speech_commander.say(input_robot_text)

        while rclpy.ok():
            try:
                # wait for julius
                self.__julius_text = ""
                jtalk_retry_time = 0
                self.node.get_logger().info("Wait for Julius text")

                while jtalk_retry_time < self.__max_jtalk_retry_time:
                    start_time = self.node.get_clock().now()
                    while self.__julius_text == "":
                        rclpy.spin_once(self.node, timeout_sec=0.1)
                        if (self.node.get_clock().now() - start_time) > self.timeout_duration:
                            self.node.get_logger().warn("Juliusからの応答がありませんでした。")
                            jtalk_retry_time += 1
                            break
                    if self.__julius_text != "":
                        break
                if self.__julius_text == "":
                    self.node.get_logger().error("Juliusからの返答がないため終了します")

                input_user_text = self.__julius_text
                self.node.get_logger().info(input_user_text)
                if image is None:
                    self.__chat_history.append({"role": "user", "content": input_user_text})
                else:
                    _, bin_image = cv2.imencode('.png', image)
                    base64_image = base64.b64encode(bin_image).decode('utf-8')
                    content = [
                        {"type": "text", "text": input_user_text},
                        {"type": "image_url", "image_url": f"data:image/jpeg;base64,{base64_image}"},
                    ]
                    self.__chat_history.append({"role": "user", "content": content})

                if image is None:
                    result_json = self.__speaker.get_response(self, image=image, contexts=self.__chat_history)
                else:
                    result_json = self.__speaker.get_response_use_image(self, contents=self.__chat_history)
                response_data = json.loads(result_json)
                if 'end_conversation' not in response_data:
                    if 'speech_phrase' in response_data:
                        speech_commander.say(response_data['speech_phrase'])
                        return result_json
                if 'end_conversation' in response_data and response_data['end_conversation']:
                    if 'speech_phrase' in response_data:
                        speech_commander.say(response_data['speech_phrase'])
                        return result_json
                if 'speech_phrase' in response_data:
                    speech_commander.say(response_data['speech_phrase'])
                    self.__chat_history.append({"role": "assistant", "content": response_data['speech_phrase']})
            except json.JSONDecodeError:
                self.node.get_logger().error("JSON解析エラーが発生しました。")
                break
            except Exception as e:
                self.node.get_logger().error(f"予期せぬエラーが発生しました: {str(e)}")
                break
        self.__chat_history = [{"role": "system", "content": self.__setting_file}]
