#!/usr/bin/env python
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
"""GPT chat"""

import base64
import os
import pathlib
import typing

import cv2
import np
import openai
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node


class GPTClient(object):
    """Chat GPT based chat bot class."""

    def __init__(self,
                 node: Node,
                 api_key: str = '',
                 model_name: str = 'gpt-4-turbo',
                 use_image: bool = False,
                 setting_file: typing.Optional[pathlib.Path] = None) -> None:
        """Constructor.

            Args:
                api_key: API key.
                model_name: Name of the model to use.
                setting_file: Settings text file path.
                optical_setting_file: Optical Setting file.
              """

        self.node = node
        if api_key is None or api_key == "":
            if os.getenv('OPENAI_API_KEY', '') == "":
                self.node.get_logger().error("No api_key...")
                return False
            else:
                self.node.get_logger().warn("Use env OPENAI_API_KEY")

        self.__model_name = model_name

        if setting_file is not None:
            self.__setting_file = setting_file
        else:
            self.__setting_file = """
                出力はjson形式で{"speech_phrase": ""}のように、speech_phraseの中に返答をいれて返してください"""

        if not use_image:
            self.__client = openai.OpenAI(api_key=api_key)
        else:
            self.__client = openai.OpenAI(api_key=api_key)

    def get_response_use_image(self, input_text: str = None, image: np.ndarray = None, contexts: dict = None) -> str:
        if image is None:
            self.node.get_logger().warn("use test image...")
            package_path = get_package_share_directory('cube_petit_python_api')
            image_path = f"{package_path}/resource/cube_petit_cad.png"
            image = cv2.imread(image_path)
            if image is None:
                raise FileNotFoundError(f"Image not found at {image_path}")

        _, bin_image = cv2.imencode('.png', image)
        base64_image = base64.b64encode(bin_image).decode('utf-8')

        if contexts is None:
            if input_text is None:
                self.node.get_logger().error("No input_text")
                return False
            contexts = [
                {"role": "system", "content": self.__setting_file},
                {"role": "user",
                 "content": [
                     {"type": "text", "text": input_text},
                     {"type": "image_url", "image_url": f"data:image/jpeg;base64,{base64_image}"},
                 ],
                 }
            ]
        response = self.__client.chat.completions.create(
            model="gpt-4-vision-preview",
            response_format={'type': 'json_object'},
            messages=contexts,
            max_tokens=300,
        )
        self.node.get_logger().info(response.choices[0].message.content)
        return response.choices[0].message.content

    def get_response(self, input_text: str = None, contexts: dict = None) -> str:
        """Sends request via Chat-GPT API.

        Args:
            input_text: User input text.

        Returns:
            Response message.
        """
        if contexts is None:
            if input_text is None:
                self.node.get_logger().error("No input_text")
                return False
            contexts = [
                {"role": "system", "content": self.__setting_file},
                {"role": "user", "content": input_text}
            ]
        result = self.__client.chat.completions.create(
            model=self.__model_name,
            response_format={'type': 'json_object'},
            messages=contexts,
            max_tokens=1000)
        self.node.get_logger().info(result.choices[0].message.content)

        return result.choices[0].message.content
