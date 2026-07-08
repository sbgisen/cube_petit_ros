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
"""Thin wrapper of navigation.launch.py for the pink robot.

nav2_params_pink.yaml is kept as a robot-specific file because it differs from the shared
nav2_params.yaml in values (e.g. keepout filter applied to the local costmap), not only in
the robot namespace.
"""
import pathlib

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Generate launch descriptions.

    Returns:
        Launch descriptions
    """
    pkg_share = pathlib.Path(FindPackageShare('cube_petit_navigation').find('cube_petit_navigation'))
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(pkg_share / 'launch/navigation.launch.py')),
            launch_arguments={
                'robot': 'cube_petit_pink',
                'params_file': str(pkg_share / 'config/nav2_params_pink.yaml'),
            }.items(),
        ),
    ])
