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
#

import glob

from setuptools import find_packages
from setuptools import setup

package_name = 'cube_petit_diagnostics'

setup(name=package_name,
      version='0.0.0',
      packages=find_packages(),
      data_files=[
          ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
          ('share/' + package_name, ['package.xml']),
          (f'share/{package_name}/launch', glob.glob('./launch/*.launch.py')),
          (f'share/{package_name}/config', glob.glob('./config/*.yaml')),
      ],
      maintainer='gisen',
      maintainer_email='SBGRP-git@g.softbank.co.jp',
      description='The cube_petit_diagnostics package',
      license='Apache License, Version2.0',
      tests_require=['pytest'],
      entry_points={
        'console_scripts': [
            'topic_watchdog_node = cube_petit_diagnostics.topic_watchdog_node:main',
            'node_watchdog_node = cube_petit_diagnostics.node_watchdog_node:main',
        ],
    })
