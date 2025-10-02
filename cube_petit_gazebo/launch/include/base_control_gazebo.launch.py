#!/usr/bin/env python
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

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import SetParameter



def generate_launch_description() -> LaunchDescription:
    """Generate launch descriptions.

    Returns:
        Launch descriptions
    """
    load_joint_state_controller = Node(package='controller_manager',
                                       executable='spawner',
                                       output='both',
                                       arguments=['-c', '/controller_manager',
                                                  'joint_state_broadcaster'])

    load_diff_drive_controller = Node(package='controller_manager',
                                      executable='spawner',
                                      output='both',
                                      arguments=['-c', '/controller_manager',
                                                 'diff_drive_controller'])

    return LaunchDescription([
        SetParameter(name='use_sim_time', value=True),
        load_joint_state_controller,
        load_diff_drive_controller,
    ])
