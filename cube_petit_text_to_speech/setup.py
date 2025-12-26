#!/usr/bin/env python

# Copyright (c) 2025 SoftBank Corp.
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
"""Install script."""

from glob import glob
import os
import subprocess

from setuptools import setup

package_name = 'cube_petit_text_to_speech'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (f'share/{package_name}', ['package.xml']),
        (f'share/{package_name}', glob('launch/*launch.py')),
        (f'share/{package_name}/config', glob('./config/*')),
        (f'share/{package_name}', ['pyproject.toml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gisen',
    maintainer_email='SBGRP-git@g.softbank.co.jp',
    description='The cube_petit_text_to_speech package',
    license='',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'speech_action_server = cube_petit_text_to_speech.speech_action_server:main',
            'cube_petit_text_to_jtalk = cube_petit_text_to_speech.cube_petit_text_to_jtalk:main',
        ]
    },
)

# Set install path to create symbolic link for speech library files.
src_path = os.path.dirname(os.path.realpath(__file__))
sym_path = os.path.join(src_path, '..', '..', '..', 'install', package_name, 'share', package_name)

subprocess.run([os.path.join(src_path, 'setup/install-jtalk.sh')],
               stdout=subprocess.DEVNULL,
               stderr=subprocess.DEVNULL)

# Create symbolic link.
symlink_path = os.path.join(sym_path, 'speech_lib')
target_path = os.path.join(src_path, 'setup', 'speech_lib')
if os.path.islink(symlink_path) or os.path.exists(symlink_path):
    os.remove(symlink_path)

os.symlink(target_path, symlink_path)

subprocess.Popen([f'{package_name}/fix_shebang.py'],
                 stdout=subprocess.DEVNULL,
                 stderr=subprocess.DEVNULL,
                 stdin=subprocess.DEVNULL,
                 start_new_session=True)
