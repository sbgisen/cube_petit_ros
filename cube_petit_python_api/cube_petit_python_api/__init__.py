#!/usr/bin/env python
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
"""Python API for cube_petit.

All submodules and classes are exported lazily (PEP 562) so that importing
``cube_petit_python_api`` itself never pulls in ROS dependencies. This keeps
the pure-logic helpers usable (and testable) in non-ROS environments.
"""

import importlib
import typing

_LAZY_MODULES = ('commanders', 'cube_petit_commander', 'exceptions', 'petit', 'petit_names', 'utils')
_LAZY_ATTRS = {
    'CubePetit': ('petit', 'CubePetit'),
    'CubePetitClosed': ('exceptions', 'CubePetitClosed'),
    'CubePetitCommander': ('cube_petit_commander', 'CubePetitCommander'),
    'CubePetitError': ('exceptions', 'CubePetitError'),
    'CubePetitNotRunning': ('exceptions', 'CubePetitNotRunning'),
    'RobotPose': ('petit_names', 'RobotPose'),
}

__all__ = list(_LAZY_MODULES) + list(_LAZY_ATTRS)


def __getattr__(name: str) -> typing.Any:  # noqa: ANN401
    """Import submodules and public classes on first access.

    Args:
        name: Attribute name.

    Returns:
        The submodule or class.

    Raises:
        AttributeError: If the attribute is unknown.
    """
    if name in _LAZY_MODULES:
        return importlib.import_module(f'{__name__}.{name}')
    if name in _LAZY_ATTRS:
        module_name, attr = _LAZY_ATTRS[name]
        return getattr(importlib.import_module(f'{__name__}.{module_name}'), attr)
    raise AttributeError(f'module {__name__!r} has no attribute {name!r}')
