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

import os
import sys

import eff_word_net.audio_processing
from yaml import KeyToken
import numpy as np
import onnxruntime
import rclpy
from rclpy.node import Node
from eff_word_net.audio_processing import Resnet50_Arc_loss
from eff_word_net.engine import HotwordDetector
from eff_word_net.engine import MultiHotwordDetector
from eff_word_net.streams import SimpleMicStream
from std_msgs.msg import Empty, String

class Resnet50ArcLossOption(Resnet50_Arc_loss):
    """ EfficientNet hotword detector
        When the node detect the hotword, publish to ~detect and ~detect_word
    """

    def __init__(self) -> None:
        self.window_length = 1.5
        self.window_frames = int(self.window_length * 16000)

        options = onnxruntime.SessionOptions()
        options.intra_op_num_threads = 1

        self.onnx_sess = onnxruntime.InferenceSession(
            os.path.join(
                eff_word_net.audio_processing.LIB_FOLDER_LOCATION,
                "models/resnet_50_arc/slim_93%_accuracy_72.7390%.onnx"),
            sess_options=options,
            providers=["CPUExecutionProvider"])

        self.input_name: str = self.onnx_sess.get_inputs()[0].name
        self.output_name: str = self.onnx_sess.get_outputs()[0].name

        self.audioToVector(np.float32(np.zeros(self.window_frames,)))  # warmup inference


eff_word_net.audio_processing.MODEL_TYPE_MAPPER["resnet_50_arc"] = Resnet50ArcLossOption


class HotwordDetectorRos(Node):
    """ EfficientNet hotword detector
        When the node detect the hotword, publish to ~detect and ~detect_word
    """

    def __init__(self) -> None:
        super().__init__('efficientword_net_hotword_detector')
        self.__pub_detect = self.create_publisher(Empty, 'detect', 10)
        self.__pub_detect_word = self.create_publisher(String, 'detect_word', 10)
        self.get_logger().info("Init")
        #[TODO] Param
        self.declare_parameter("keywords", ["Cube_petit"])
        self.declare_parameter("thresholds", [0.65])

        # Loading the ~keywords' list from the parameter server
        if self.has_parameter('keywords'):
            keywords = self.get_parameter('keywords').value
            self.get_logger().info(f"Read keywords list: {keywords}")
        else:
            self.get_logger().error("Parameter 'keywords' not found!")
            sys.exit(1)
        # Checking if keywords are strings
        if not all(isinstance(item, str) for item in keywords):
            self.get_logger().error("keywords must be string")
            sys.exit(1)

        # Loading the 'thresholds' list from the parameter server
        if self.has_parameter('thresholds'):
            thresholds = self.get_parameter('thresholds').value
            self.get_logger().info(f"Read thresholds list: {thresholds}")
        else:
            self.get_logger().error("Parameter 'thresholds' not found!")
            sys.exit(1)
        # Checking if thresholds are numeric values between 0 and 1 (0.XX)
        if not all(0 <= item <= 1 for item in thresholds):
            self.get_logger().error("thresholds must be between 0 and 1")
            sys.exit(1)

        # Checking if the number of elements in keywords and thresholds is the same
        if len(keywords) != len(thresholds):
            self.get_logger().error("(keywords, thresholds) these lists have different numbers of elements")
            sys.exit(1)

        #[TODO]Param
        # base_model_str = self.get_parameter('base_model', 'resnet50').value
        base_model_str = 'resnet50'
        base_model = Resnet50ArcLossOption()

        # base_model (resnet50) [TODO] First_Iteration_Siamese, ModelRawBackend
        if base_model_str != 'resnet50':
            self.get_logger().error("You can only use resnet50 in the current")
            sys.exit(1)

        # Creating a dictionary of instances for EfficientWordNet
        efficient_word_net_list = []
        for keyword, threshold in zip(keywords, thresholds):
            self.get_logger().debug(f"keyword name: {keyword}")
            #[TODO] param
            script_dir = os.path.dirname(os.path.abspath(__file__))
            keyword_path = os.path.join(script_dir, '../../share/cube_petit_speech_to_text/resources/Cube-petit_ref.json')

            if os.path.exists(keyword_path):
                self.get_logger().debug(f"keyword_path: {keyword_path}")
            else:
                self.get_logger().error(f"File does not exist: {keyword_path}")
                sys.exit(1)
            self.get_logger().debug(f"threshold {threshold}")
            self.efficient_word_net = HotwordDetector(hotword=keyword,
                                                      model=base_model,
                                                      reference_file=keyword_path,
                                                      threshold=threshold)
            efficient_word_net_list.append(self.efficient_word_net)

        # If there are multiple keywords, use the multi_hotword_detector
        self.num_keywords = len(keywords)
        if self.num_keywords > 1:
            self.get_logger().debug(f"keywords: {self.num_keywords}, MultiHotword Mode")
            self.multi_hotword_detector = MultiHotwordDetector(
                efficient_word_net_list,
                model=base_model,
                continuous=True,
            )

    def run(self) -> None:
        """ Run hotword detector
        """
        mic_stream = SimpleMicStream(window_length_secs=1.5, sliding_window_secs=0.75)
        mic_stream.start_stream()

        # single keyword detector
        if self.num_keywords == 1:
            self.get_logger().info(f"Say {self.efficient_word_net.hotword}")
            while rclpy.ok():
                frame = mic_stream.getFrame()
                result = self.efficient_word_net.scoreFrame(frame)
                if result is None:
                    # no voice activity
                    continue
                if result["match"]:
                    self.get_logger().info(f"Hotword detected, Confidence : {result['confidence']}")
                    msg = String(data=self.efficient_word_net.hotword)
                    self.__pub_detect.publish(Empty())
                    self.__pub_detect_word.publish(msg)
        else:
            self.get_logger().info(f"Say {' / '.join([x.hotword for x in self.multi_hotword_detector.detector_collection])}")
            while rclpy.ok():
                frame = mic_stream.getFrame()
                result = self.multi_hotword_detector.findBestMatch(frame)
                if None not in result:
                    # [TODO] Fix an error when there are insufficient arguments
                    result_str = str(result[0]).split(":")[-1].strip()
                    self.get_logger().info(f"Hotword detected word: {result_str}, Confidence : {result[1]}")
                    msg = String(data=result_str)
                    self.__pub_detect.publish(Empty())
                    self.__pub_detect_word.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = HotwordDetectorRos()
    node.run()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
