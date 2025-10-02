#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright (c) 2022 SoftBank Corp.
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
"""Collections of function that commands cube."""

import rclpy
from beartype import beartype
from rclpy.exceptions import NotInitializedException
from rclpy.node import Node

try:
    from cube_petit_python_api.commanders.speech import SpeechCommander
except ImportError:
    pass

SPEECH = 'SPEECH'


@beartype
class CubePetitCommander(object):
    """Collections of commands to access and manage cube functionalities.

    Cube Commander is an efficient method of access to cube's commanders/functions when multiple
    instantiations refer to the same commanders/functions. Commanders are loaded (instantiated) once
    to be access and refer to at all times regardless of multiple Cube Commander instantiations
    or function calls, as long as it is in a single rosnode.

    Commanders/functions can be accessed either through Cube Commander instantiations or direct
    function calls. Instantiating Cube Commander will load all available commanders while directly
    calling the functions will load only required commanders.
    """
    commander = {
        SPEECH: SpeechCommander if 'SpeechCommander' in globals() else None
    }

    def __init__(self, node: Node) -> None:
        """Load all available commanders.

        Args:
            node: ROS node.
        """
        self._node = node
        for key, value in self.commander.items():
            if isinstance(value, type):
                try:
                    self.commander[key] = value(node)
                    self._node.get_logger().info(f'Loaded {key} commander')
                except NotInitializedException as e:
                    self._node.get_logger().warn(f'Unable to load {key} commander because {e}')
            elif value is None:
                self._node.get_logger().warn(f'Unable to load {key} commander because it is not installed')

    # @classmethod
    # @load_commanders(SPEECH)
    def say(
            self,
            phrase_id: str,
            speech_method: str | None = None,
            emotion: str | None = None,
            emotion_level: int | None = None,
            pitch: int | None = None,
            speed: int | None = None,
            volume: int | None = None,
            sync: bool = True,
            wait_for_previous: bool = False,
            until_success: bool = False) -> bool:
        """Make speech.

        Args:
            phrase_id: The ID representing the phrase to be said. Search for a rosparam
                       with the key `/speech_info/speech/{phrase_id}/{language}` and
                       give its value to the speech server (`language` can be set by
                       rosparam `/speech_info/language`). If not found, give the
                       `phrase_id` itself to the speech server.
            speech_method: Name of the speech method to be used. Defaults to None.
            emotion: The emotion used for the speech. Used in jtalk and voicetext. Defaults to None.
            emotion_level: The level of emotion to be used. Used in voicetext only. Defaults to None.
            pitch: The pitch of the speech. Defaults to None.
            speed: The speed of the speech. Defaults to None.
            volume: The volume of the speech. Defaults to None.
            sync: Whether to wait for the speech to end or not. Defaults to True.
            wait_for_previous: Whether to wait for the previous asynchronous speech to end or not. Defaults to False.

        Raises:
            rospy.ROSInitException: rospy.init_node() is not called or unable to load SpeechCommander.

        Returns:
            True if a speech has succeeded or is disabled or is held asynchronously.
        """
        result = CubePetitCommander.commander[SPEECH].say(
            phrase_id, speech_method, emotion, emotion_level,
            pitch, speed, volume, sync, wait_for_previous)
        if not result and until_success:
            while rclpy.ok():
                self._node.get_logger().info('Retrying speech')
                result = cls.say(
                    phrase_id, speech_method, emotion, emotion_level,
                    pitch, speed, volume, sync, wait_for_previous)
                if result:
                    break
        return result
