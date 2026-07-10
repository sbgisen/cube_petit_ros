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
import signal
import subprocess
import tempfile

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

COLOR_MAP = {
    'blue': '#42AFE3',
    'pink': '#ff7da8',
    'orange': '#ffa500',
    'green': '#6ac259',
    'yellow': '#fff176',
    'purple': '#ba68c8',
    'red': '#ef5350',
    'clear': '#A3A3A3',
    'lightgreen': '#42E34D',
    'white': '#8E8E8E',
}
script_dir = os.path.dirname(os.path.abspath(__file__))
html_path = os.path.join(script_dir, '../../share/cube_petit_facial_animation/frontend/index.html')

# Dedicated Chrome profile directory for this node's Chrome instance. Without this,
# google-chrome's single-instance behavior means that if a Chrome browser is already
# running for this user (default profile dir ~/.config/google-chrome), our Popen call
# just forwards a "open a new --guest --kiosk window" request to that *existing*
# browser process over IPC and immediately exits — our Popen handle then points at a
# short-lived helper process, not at the real window, so kill_started() cannot reliably
# find or stop the actual on-screen window, and the forwarded --guest/--kiosk request
# can disrupt (or crash) whatever the user was already doing in that browser. Using a
# separate --user-data-dir sidesteps single-instance detection entirely, so this node
# always gets its own independent Chrome process tree to start and stop.
chrome_user_data_dir = os.path.join(tempfile.gettempdir(), 'cube_petit_facial_animation_chrome_profile')


class Chrome():
    """Chrome process."""

    def __init__(self) -> None:
        """Init."""
        self._p = None

    def start(self) -> None:
        """Start Chrome."""
        # start_new_session=True puts Chrome (and the process tree it spawns for its
        # renderer/GPU helper processes) in its own process group, so kill_started()
        # can terminate the whole group instead of only the top-level process.
        self._p = subprocess.Popen([
            'google-chrome', '--new-window', '--start-fullscreen', '--disable-features=Translate', '--guest',
            '--kiosk', '--start-maximized', '--password-store=basic', '--no-default-browser-check',
            '--hide-crash-restore-bubble', f'--user-data-dir={chrome_user_data_dir}', html_path
        ],
                                   start_new_session=True)

    def kill_started(self) -> None:
        """Stop Chrome."""
        # No-op if Chrome was never started (e.g. start() failed before this is called).
        if self._p is None:
            return
        try:
            pgid = os.getpgid(self._p.pid)
        except ProcessLookupError:
            # Already gone.
            return
        try:
            os.killpg(pgid, signal.SIGTERM)
            self._p.wait(timeout=5)
        except ProcessLookupError:
            # Already gone.
            pass
        except subprocess.TimeoutExpired:
            # Still alive after SIGTERM, force kill the whole process group.
            try:
                os.killpg(pgid, signal.SIGKILL)
            except ProcessLookupError:
                pass

    def kill_all(self) -> None:
        """Kill all chrome process."""
        # Currently unused.
        # Use this to kill an already-running Chrome instance at rosnode startup, if needed.
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
        pass  # This timer callback does nothing, but keeps a 10Hz loop running.

    def shutdown(self) -> None:
        """Shutdown."""
        self.chrome.kill_started()


def main() -> None:
    rclpy.init()
    node = ChromeNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # These are the normal exit paths when launch sends SIGINT: spin() raises
        # instead of returning, so node.shutdown() must run in finally below to
        # actually happen (this was the bug: Chrome was never killed on stop).
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
