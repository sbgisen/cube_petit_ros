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

import os
import subprocess

import rclpy
from rclpy.node import Node

COLOR_MAP = {
    'blue': '#42AFE3',
    'pink': '#ff7da8',
    'orange': '#ffa500',
    'green': '#6ac259',
    'yellow': '#fff176',
    'purple': '#ba68c8',
    'red': '#ef5350',
    'clear': "#A3A3A3",
    'lightgreen': "#42E34D",
    'white': "#8E8E8E",
}
script_dir = os.path.dirname(os.path.abspath(__file__))
html_path = os.path.join(script_dir, '../../share/cube_petit_facial_animation/frontend/index.html')


class Chrome():
    """Chrome process."""

    def start(self) -> None:
        """Start Chrome."""
        self._p = subprocess.Popen([
            'google-chrome', '--new-window', '--start-fullscreen', '--disable-features=Translate', '--guest',
            '--kiosk', '--start-maximized', '--password-store=basic', '--no-default-browser-check',
            '--hide-crash-restore-bubble', html_path
        ])

    def kill_started(self) -> None:
        """Stop Chrome."""
        # rosnode開始時にすでにChromeが起動していた場合は何も起きない
        self._p.terminate()

    def kill_all(self) -> None:
        """Kill all chrome process."""
        # 現在は未使用
        # rosnode開始時、起動済みのChromeを終了させたい場合に使用
        subprocess.call(['pkill', '-f', '/opt/google/chrome/chrome'])


class ChromeNode(Node):

    def __init__(self) -> None:
        """Init."""
        super().__init__('cube_facial_animation')
        self.declare_parameter('color', 'blue')
        color_name = self.get_parameter('color').get_parameter_value().string_value
        color_code = COLOR_MAP.get(color_name, '#42AFE3')
        self.apply_color_to_css(color_code)

        self.chrome = Chrome()
        self.chrome.start()
        self.create_timer(0.1, self.timer_callback)

    def apply_color_to_css(self, color_code: str) -> None:
        """Apply color to css."""
        style_template_path = os.path.join(script_dir,
                                           '../../share/cube_petit_facial_animation/frontend/style.css.template')
        style_output_path = os.path.join(script_dir, '../../share/cube_petit_facial_animation/frontend/style.css')

        with open(style_template_path, 'r', encoding='utf-8') as f:
            css_data = f.read()

        css_data = css_data.replace('{{COLOR}}', color_code)

        with open(style_output_path, 'w', encoding='utf-8') as f:
            f.write(css_data)

    def timer_callback(self) -> None:
        """Check timer."""
        pass  # このタイマーコールバックは何もしませんが、10Hzのループを維持します

    def shutdown(self) -> None:
        """Shutdown."""
        self.chrome.kill_started()


def main() -> None:
    rclpy.init()
    node = ChromeNode()
    rclpy.spin(node)
    node.shutdown()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
