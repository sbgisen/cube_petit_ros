#!/usr/bin/env python3
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
u"""Chrome表示用rosnode"""

import os
import subprocess

import rclpy
from rclpy.node import Node

script_dir = os.path.dirname(os.path.abspath(__file__))
html_path = os.path.join(script_dir, '../../share/cube_petit_facial_animation/frontend/index.html')


class Chrome():
    u"""Chromeのプロセス管理クラス"""

    def start(self):
        u"""Chromeを起動"""
        self._p = subprocess.Popen([
            "google-chrome", "--new-window", "--start-fullscreen",
            "--no-default-browser-check", "--hide-crash-restore-bubble", html_path])

    def kill_started(self):
        u"""起動したChromeを停止"""
        # rosnode開始時にすでにChromeが起動していた場合は何も起きない
        self._p.terminate()

    def kill_all(self):
        u"""Chromeのプロセスを全て終了"""
        # 現在は未使用
        # rosnode開始時、起動済みのChromeを終了させたい場合に使用
        subprocess.call(["pkill", "-f", "/opt/google/chrome/chrome"])


class ChromeNode(Node):
    def __init__(self):
        super().__init__('cube_facial_animation')
        self.chrome = Chrome()
        self.chrome.start()
        self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        pass  # このタイマーコールバックは何もしませんが、10Hzのループを維持します

    def shutdown(self):
        self.chrome.kill_started()


def main(args=None):
    rclpy.init(args=args)
    node = ChromeNode()
    rclpy.spin(node)
    node.shutdown()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
