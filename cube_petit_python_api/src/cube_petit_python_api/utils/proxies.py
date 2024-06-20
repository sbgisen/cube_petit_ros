#!/usr/bin/env python
# -*- coding:utf-8 -*-

# Copyright (c) 2023 SoftBank Corp.
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

from beartype import beartype
from beartype.typing import Type
from rclpy.action import ActionClient
from rclpy.callback_groups import CallbackGroup
from rclpy.client import Client
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.qos import QoSProfile
from rclpy.qos import qos_profile_action_status_default
from rclpy.qos import qos_profile_services_default
from rclpy.qos_event import PublisherEventCallbacks
from rclpy.qos_overriding_options import QoSOverridingOptions


@beartype
def get_publisher(node: Node,
                  msg_type: type,
                  topic: str,
                  qos_profile: QoSProfile | int,
                  *,
                  callback_group: CallbackGroup | None = None,
                  event_callbacks: PublisherEventCallbacks | None = None,
                  qos_overriding_options: QoSOverridingOptions | None = None,
                  publisher_class: Type[Publisher] = Publisher) -> Publisher:
    """Get/Create publisher from node.

    Args:
        node: ROS2 node to get publisher from.
        msg_type: Publishing message type.
        topic: Topic name.
        qos_profile: QoS profile.
        callback_group: Callback group. Defaults to None.
        event_callbacks: Event callbacks. Defaults to None.
        qos_overriding_options: QoS overriding options. Defaults to None.
        publisher_class: Publisher class. Defaults to Publisher.

    Returns:
        Publisher instance.
    """
    pub = filter(lambda x: x.topic == topic, node.publishers)
    try:
        return next(pub)
    except StopIteration:
        return node.create_publisher(msg_type,
                                     topic,
                                     qos_profile,
                                     callback_group=callback_group,
                                     event_callbacks=event_callbacks,
                                     qos_overriding_options=qos_overriding_options,
                                     publisher_class=publisher_class)


@beartype
def get_client(node: Node,
               srv_type: type,
               srv_name: str,
               *,
               qos_profile: QoSProfile = qos_profile_services_default,
               callback_group: CallbackGroup | None = None) -> Client:
    """Get/Create service client from node.

    Args:
        node: ROS2 node to get client from.
        srv_type: Service message type.
        srv_name: Service name.
        qos_profile: QoS profile. Defaults to qos_profile_services_default.
        callback_group: Callback group. Defaults to None.

    Returns:
        Service client instance.
    """
    client = filter(lambda x: x.srv_name == srv_name, node.clients)
    try:
        return next(client)
    except StopIteration:
        return node.create_client(srv_type,
                                  srv_name,
                                  qos_profile=qos_profile,
                                  callback_group=callback_group)


@beartype
def get_action_client(node: Node,
                      action_type: type,
                      action_name: str,
                      *,
                      callback_group: CallbackGroup | None = None,
                      goal_service_qos_profile: QoSProfile = qos_profile_services_default,
                      result_service_qos_profile: QoSProfile = qos_profile_services_default,
                      cancel_service_qos_profile: QoSProfile = qos_profile_services_default,
                      feedback_sub_qos_profile: QoSProfile = QoSProfile(depth=10),
                      status_sub_qos_profile: QoSProfile = qos_profile_action_status_default
                      ) -> ActionClient:
    """Get/Create action client from node.

    Args:
        node: ROS2 node to get action client from.
        action_type: Action message type.
        action_name: Action name.
        callback_group: Callback group. Defaults to None.
        goal_service_qos_profile: QoS profile for goal service. Defaults to qos_profile_services_default.
        result_service_qos_profile: QoS profile for result service. Defaults to qos_profile_services_default.
        cancel_service_qos_profile: QoS profile for cancel service. Defaults to qos_profile_services_default.
        feedback_sub_qos_profile: QoS profile for feedback subscription. Defaults to QoSProfile(depth=10).
        status_sub_qos_profile: QoS profile for status subscription. Defaults to qos_profile_action_status_default.

    Returns:
        Action client instance.
    """
    if not hasattr(node, 'action_clients'):
        node.action_clients = {}

    if action_name not in node.action_clients:
        node.action_clients[action_name] = ActionClient(node,
                                                        action_type,
                                                        action_name,
                                                        callback_group=callback_group,
                                                        goal_service_qos_profile=goal_service_qos_profile,
                                                        result_service_qos_profile=result_service_qos_profile,
                                                        cancel_service_qos_profile=cancel_service_qos_profile,
                                                        feedback_sub_qos_profile=feedback_sub_qos_profile,
                                                        status_sub_qos_profile=status_sub_qos_profile)

    return node.action_clients[action_name]
