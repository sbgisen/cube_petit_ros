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

import glob
import subprocess

from setuptools import find_packages
from setuptools import setup

package_name = 'cube_petit_speech_to_text'

setup(
    name=package_name,
      version='0.0.0',
      packages=find_packages(exclude=['test']),
      install_requires=['setuptools'],
      data_files=[
          ('share/ament_index/resource_index/packages', ['resources/' + package_name]),
          ('share/' + package_name, ['package.xml']),
          (f'share/{package_name}/launch', glob.glob('./launch/*.launch.py')),
          (f'share/{package_name}/config', glob.glob('./config/*.yaml')),
          (f'share/{package_name}/resources/', glob.glob('./resources/*.json')),
          (f'share/{package_name}', ['pyproject.toml']),
      ],
      maintainer='gisen',
      maintainer_email='SBGRP-git@g.softbank.co.jp',
      description='The cube_petit_speech_to_text package',
      license='Apache License, Version2.0',
      tests_require=['pytest'],
      entry_points={
          'console_scripts': [f'hotword_detector = {package_name}.cube_petit_hotword_detector:main',
                              f'cube_petit_speech_to_text = {package_name}.cube_petit_speech_to_text:main',]          
      }
    )

# バックグラウンドプロセスを実行している箇所
subprocess.Popen([f'{package_name}/fix_shebang.py'],
                 stdout=subprocess.DEVNULL,
                 stderr=subprocess.DEVNULL,
                 stdin=subprocess.DEVNULL,
                 start_new_session=True)