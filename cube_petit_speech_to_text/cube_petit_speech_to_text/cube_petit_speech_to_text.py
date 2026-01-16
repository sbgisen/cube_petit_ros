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

import os
import signal
import subprocess
import sys

import rclpy
import rclpy.node
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import String


class JuliusSpeechToText(rclpy.node.Node):
    def __init__(self):
        super().__init__('cube_petit_speech_to_text')
        self.publisher_ = self.create_publisher(String, 'julius_result_text', 1)
        self.rate = self.create_rate(1)  # 1hz
        self.pkill_julius()
        self.start_listening()

    def __del__(self):
        self.stop_listening()
        self.pkill_julius()

    def start_listening(self):
        cmd = 'cd ~/work/julius_libs/dictation-kit/;bash run-linux-dnn.sh'
        self.julius_process = subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE, shell=True)
        self.get_logger().info('julius:started')

    def stop_listening(self):
        self.get_logger().info('julius:successfully finished')
        self.julius_process.kill()

    def pkill_julius(self):
        p = subprocess.Popen(['pgrep', '-l', 'julius'], stdout=subprocess.PIPE)
        out, err = p.communicate()
        for line in out.splitlines():
            line = line.decode()
            pid = int(line.split(None, 1)[0])
            os.kill(pid, signal.SIGKILL)

    def text_streaming(self):
        for line in iter(self.julius_process.stdout.readline, b""):
            out_string = line.decode().replace("\n", " ")
            print(out_string)
            find_text_flag = out_string.find('sentence1:')
            find_pass_flag = out_string.find('pass1_best:')
            find_short_text_flag = out_string.find('<input rejected by short input>')
            if find_text_flag != -1 and find_short_text_flag == -1:
                text_string = out_string.replace("sentence1:", "")
                text_string = text_string.replace(" ", "")
                text_string = text_string.replace("。", "")
                if text_string != '。' and len(text_string) > 3:
                    self.publisher_.publish(String(data=text_string))
                    print(text_string)
            elif find_pass_flag != -1 and find_short_text_flag != -1:
                text_string = out_string.replace("pass1_best:", "")
                text_string = text_string.replace("<input rejected by short input>", "")
                text_string = text_string.replace(" ", "")
                text_string = text_string.replace("。", "")
                if text_string != '。' and len(text_string) > 3:
                    self.publisher_.publish(String(data=text_string))
                    print(text_string)

def main():
    rclpy.init()
    try:
        node = JuliusSpeechToText()
        node.text_streaming()
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
