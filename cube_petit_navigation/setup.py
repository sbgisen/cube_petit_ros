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

package_name = 'cube_petit_navigation'

setup(name=package_name,
      version='0.0.0',
      packages=find_packages(),
      data_files=[
          ('share/' + package_name, ['package.xml']),
          (f'share/{package_name}/launch', glob.glob('./launch/*.launch.py')),
          (f'share/{package_name}/config', glob.glob('./config/*.yaml')),
          (f'share/{package_name}/map/rooms', glob.glob('./map/rooms/*')),
          (f'share/{package_name}/map/sample', glob.glob('./map/sample/*')),
        (f'share/{package_name}/map/test', glob.glob('./map/test/*')),
      ],
      install_requires=['setuptools'],
      maintainer='gisen',
      maintainer_email='SBGRP-git@g.softbank.co.jp',
      description='cube_petit_navigation package',
      license='Apache License, Version2.0',
      tests_require=['pytest'],
      entry_points={
          'console_scripts': [f'navigation_api_node = {package_name}.navigation_api_node:main',],
      })
