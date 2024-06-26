#!/usr/bin/env python
# -*- coding: utf-8 -*-

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
"""Cube petit's speech."""

import typing

import rclpy
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.node import Node
from sbgisen_msgs.action import Speech


class SpeechCommander:
    """Commander for handling speech."""

    enable_init_timeout = True

    def __init__(self, node: Node) -> None:
        """Constructor."""
        self.node = node
        self.speaker = ActionClient(node, Speech, '/speech_action_server')
        # Wait for connection
        self.node.get_logger().info('Waiting for action server...')
        if not self.speaker.wait_for_server(timeout_sec=5.0):
            self.node.get_logger().error('Unable to find /speech_action_server.')
            raise Exception('Unable to find /speech_action_server.')
        self.node.get_logger().info("Speech Commander init")

    def say(self,
            phrase_id: str,
            emotion: typing.Optional[str] = None,
            emotion_level: typing.Optional[int] = None,
            pitch: typing.Optional[int] = None,
            speed: typing.Optional[int] = None,
            volume: typing.Optional[int] = None,
            sync: bool = True,
            wait_for_previous: bool = False) -> bool:
        """Make speech by calling speech server.

        Args:
            phrase_id: The ID representing the phrase to be said.
            emotion: The emotion used for the speech. Defaults to None.
            emotion_level: The level of emotion to be used. Defaults to None.
            pitch: The pitch of the speech. Defaults to None.
            speed: The speed of the speech. Defaults to None.
            volume: The volume of the speech. Defaults to None.
            sync: Whether to wait for the speech to end or not. Defaults to True.
            wait_for_previous: Whether to wait for the previous asynchronous speech to end or not. Defaults to False.

        Returns:
            True if a speech has succeeded or if the speech is disabled or if the speech is held asynchronously.
        """

        if not isinstance(phrase_id, str):
            raise TypeError('Invalid phrase input, use str type input for phrase or phrase_id')

        if emotion is None:
            emotion = "happiness"
        if emotion_level is None:
            emotion_level = 2
        if pitch is None:
            pitch = 100
        if speed is None:
            speed = 100
        if volume is None:
            volume = 30

        # Casting to str in case ascii value is given via parameter
        phrase = phrase_id

        goal_msg = Speech.Goal()
        goal_msg.text = phrase
        goal_msg.method = "jtalk"
        goal_msg.emotion = emotion
        goal_msg.emotion_level = emotion_level
        goal_msg.pitch = pitch
        goal_msg.speed = speed
        goal_msg.volume = volume

        self.node.get_logger().info(f"Robot say: [{phrase}]")
        self._send_goal_future = self.speaker.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

        if sync:
            rclpy.spin_until_future_complete(self.node, self._send_goal_future)
            return self._send_goal_future.result().status == GoalStatus.STATUS_SUCCEEDED

        return True

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.node.get_logger().info('Goal rejected :(')
            return

        self.node.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        if result.result:
            self.node.get_logger().info('Speech succeeded!')
        else:
            self.node.get_logger().info('Speech failed.')


def main(args=None):
    rclpy.init(args=args)
    node = Node('speech_commander_node')
    speech_commander = SpeechCommander(node)

    try:
        speech_commander.say('Hello, World!')
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
